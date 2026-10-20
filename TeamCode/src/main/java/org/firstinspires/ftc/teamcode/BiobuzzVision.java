package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * BIOBUZZ moving-HIVE AprilTag localizer for Pedro Pathing 3.
 *
 * Coordinate conventions used by this class:
 *  - Field: Pedro coordinates, inches. +X right, +Y away from audience, +Z up.
 *  - Robot: +X right, +Y forward, +Z up. Robot origin is on the tile plane.
 *  - Camera FTC pose: AprilTagPoseFtc x=right, y=forward, z=up.
 *  - HIVE angle: right-hand rotation about +field X.
 *      Positive angle raises the +Y/scoring-side CELL.
 *
 * Why this works while a HIVE is moving:
 *  1) The HIVE pivot is fixed in field coordinates.
 *  2) Each tag has a fixed neutral vector from that pivot.
 *  3) The class estimates HIVE angle from the measured tag HEIGHT (Z), which does not
 *     depend on the robot's X/Y/heading on a flat field.
 *  4) It rotates each tag's neutral vector about the pivot to recover the tag's CURRENT
 *     field position.
 *  5) It solves the robot SE(2) pose from the current field points and camera measurements.
 *
 * IMPORTANT:
 *  - Accurate camera extrinsics and tag geometry are required for accurate localization.
 *  - The default HIVE pivots are derived from the official 144-in field center,
 *    25.5-in HIVE center-to-center spacing, and 43.95-in pivot height. Verify them
 *    against the official STEP/CAD before competition and override if necessary.
 *  - Tag neutral positions are intentionally NOT guessed. Calibrate them or paste values
 *    measured from official CAD using setTagNeutralPosition().
 */
@Disabled
public final class BiobuzzVision implements AutoCloseable {

    public static final double TAG_SIZE_IN = 3.25;
    public static final int FIRST_TAG_ID = 30;
    public static final int LAST_TAG_ID = 45;

    public enum Hive {
        RED,
        BLUE
    }

    public enum CellSide {
        AUDIENCE,
        SCORING
    }

    public enum HiveState {
        AUDIENCE_UP,
        SCORING_UP,
        MOVING,
        UNKNOWN
    }

    /** Small immutable 3D vector in inches. */
    public static final class Vec3 {
        public final double x;
        public final double y;
        public final double z;

        public Vec3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vec3 plus(Vec3 other) {
            return new Vec3(x + other.x, y + other.y, z + other.z);
        }

        public Vec3 minus(Vec3 other) {
            return new Vec3(x - other.x, y - other.y, z - other.z);
        }

        public Vec3 times(double scalar) {
            return new Vec3(x * scalar, y * scalar, z * scalar);
        }

        public double norm() {
            return Math.sqrt(x * x + y * y + z * z);
        }

        public double xyNorm() {
            return Math.hypot(x, y);
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "(%.3f, %.3f, %.3f)", x, y, z);
        }
    }

    /**
     * Camera location/orientation relative to the robot origin.
     *
     * The simplest normal forward-facing upright webcam uses yaw=pitch=roll=0.
     * Translation is robot-right, robot-forward, robot-up in inches.
     *
     * Rotation convention: R = Rz(yaw) * Ry(roll) * Rx(pitch), where FTC-style
     * pitch is about +X, roll is about +Y, yaw is about +Z.
     */
    public static final class CameraExtrinsics {
        public final Vec3 translationRobot;
        private final Mat3 cameraToRobot;
        public final double yawRad;
        public final double pitchRad;
        public final double rollRad;

        private CameraExtrinsics(
                Vec3 translationRobot,
                Mat3 cameraToRobot,
                double yawRad,
                double pitchRad,
                double rollRad
        ) {
            this.translationRobot = translationRobot;
            this.cameraToRobot = cameraToRobot;
            this.yawRad = yawRad;
            this.pitchRad = pitchRad;
            this.rollRad = rollRad;
        }

        public static CameraExtrinsics inchesDegrees(
                double right,
                double forward,
                double up,
                double yawDeg,
                double pitchDeg,
                double rollDeg
        ) {
            return inchesRadians(
                    right,
                    forward,
                    up,
                    Math.toRadians(yawDeg),
                    Math.toRadians(pitchDeg),
                    Math.toRadians(rollDeg)
            );
        }

        public static CameraExtrinsics inchesRadians(
                double right,
                double forward,
                double up,
                double yawRad,
                double pitchRad,
                double rollRad
        ) {
            Mat3 rotation = Mat3.rz(yawRad)
                    .multiply(Mat3.ry(rollRad))
                    .multiply(Mat3.rx(pitchRad));
            return new CameraExtrinsics(
                    new Vec3(right, forward, up),
                    rotation,
                    yawRad,
                    pitchRad,
                    rollRad
            );
        }

        public static CameraExtrinsics forwardUpright(double right, double forward, double up) {
            return inchesRadians(right, forward, up, 0.0, 0.0, 0.0);
        }
    }

    /** Runtime/filter configuration. Tune these on your robot. */
    public static final class Config {
        public int cameraWidth = 640;
        public int cameraHeight = 480;
        public float decimation = 2.0f;

        // Optional calibrated lens intrinsics. Leave <= 0 to let FTC SDK choose defaults.
        public double fx = -1;
        public double fy = -1;
        public double cx = -1;
        public double cy = -1;

        // Detection filtering.
        public double minRangeIn = 4.0;
        public double maxRangeIn = 120.0;
        public int maxHamming = 1;
        public double minDecisionMargin = 20.0;

        // HIVE angle model/search.
        public double minHiveAngleRad = Math.toRadians(-40.0);
        public double maxHiveAngleRad = Math.toRadians(40.0);
        public double hiveAngleCoarseStepRad = Math.toRadians(0.25);
        public double hiveAngleFineStepRad = Math.toRadians(0.025);
        public double hiveAngleSmoothing = 0.35; // 0=hold old, 1=raw measurement
        public double maxHiveHeightResidualIn = 3.0;

        // Optional orientation cross-check/fusion. This uses the raw AprilTag plane normal plus
        // Pedro's current heading. Height remains available as an odometry-independent estimate.
        public boolean useTagOrientationForHiveAngle = true;
        public double orientationAngleWeight = 0.55;
        public double maxTagNormalXAbs = 0.45;
        public double maxOrientationHeightDisagreementRad = Math.toRadians(15.0);

        public long maxHiveAngleAgeMs = 400;
        public double stableAngleRad = Math.toRadians(30.0);
        public double stableToleranceRad = Math.toRadians(5.0);

        // Pose fit/outlier rejection.
        public double minHeadingBaselineIn = 4.0;
        public double maxPerTagResidualIn = 3.0;
        public double maxFitRmsIn = 2.5;

        // Gate vision against Pedro before applying.
        public double maxPositionCorrectionIn = 24.0;
        public double maxHeadingCorrectionRad = Math.toRadians(35.0);

        // Fusion. Increase after validation; 1.0 means snap to vision.
        public double singleTagPositionAlpha = 0.15;
        public double multiTagPositionAlpha = 0.35;
        public double multiTagHeadingAlpha = 0.25;
        public double minRangeScale = 0.35;

        // Do not repeatedly feed a stale camera frame to Pedro.
        public boolean useFreshDetectionsOnly = true;

        // Optional preview annotations.
        public boolean drawTagId = true;
        public boolean drawTagOutline = true;
        public boolean drawAxes = false;
        public boolean drawCube = false;
    }

    /** Result of one fresh camera-frame pose solve. */
    public static final class VisionEstimate {
        public final Pose pose;
        public final int tagsUsed;
        public final boolean headingFromVision;
        public final double rmsResidualIn;
        public final double averageRangeIn;
        public final long frameAcquisitionNanoTime;
        public final boolean passesGate;
        public final String rejectionReason;

        private VisionEstimate(
                Pose pose,
                int tagsUsed,
                boolean headingFromVision,
                double rmsResidualIn,
                double averageRangeIn,
                long frameAcquisitionNanoTime,
                boolean passesGate,
                String rejectionReason
        ) {
            this.pose = pose;
            this.tagsUsed = tagsUsed;
            this.headingFromVision = headingFromVision;
            this.rmsResidualIn = rmsResidualIn;
            this.averageRangeIn = averageRangeIn;
            this.frameAcquisitionNanoTime = frameAcquisitionNanoTime;
            this.passesGate = passesGate;
            this.rejectionReason = rejectionReason;
        }

        public double ageMs() {
            if (frameAcquisitionNanoTime <= 0) return Double.NaN;
            return (System.nanoTime() - frameAcquisitionNanoTime) / 1_000_000.0;
        }
    }

    public static final class CalibrationResult {
        public final Hive hive;
        public final int tagsCalibrated;
        public final List<Integer> tagIds;

        private CalibrationResult(Hive hive, int tagsCalibrated, List<Integer> tagIds) {
            this.hive = hive;
            this.tagsCalibrated = tagsCalibrated;
            this.tagIds = Collections.unmodifiableList(new ArrayList<>(tagIds));
        }
    }

    private static final class HiveTrack {
        double angleRad;
        boolean valid;
        long updateNanoTime;
        double lastHeightRms;
        int tagsUsed;

        HiveTrack(double seedAngleRad) {
            angleRad = seedAngleRad;
            valid = false;
            updateNanoTime = 0;
            lastHeightRms = Double.POSITIVE_INFINITY;
            tagsUsed = 0;
        }
    }

    private static final class Observation {
        final AprilTagDetection detection;
        final Vec3 robotPoint;
        final Vec3 fieldPoint;

        Observation(AprilTagDetection detection, Vec3 robotPoint, Vec3 fieldPoint) {
            this.detection = detection;
            this.robotPoint = robotPoint;
            this.fieldPoint = fieldPoint;
        }
    }

    private static final class PoseFit {
        final Pose pose;
        final boolean headingFromVision;
        final double rms;
        final List<Observation> used;

        PoseFit(Pose pose, boolean headingFromVision, double rms, List<Observation> used) {
            this.pose = pose;
            this.headingFromVision = headingFromVision;
            this.rms = rms;
            this.used = used;
        }
    }

    private final Config config;
    private CameraExtrinsics cameraExtrinsics;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private final EnumMap<Hive, Vec3> hivePivots = new EnumMap<>(Hive.class);
    private final EnumMap<Hive, HiveTrack> hiveTracks = new EnumMap<>(Hive.class);
    private final Map<Integer, Vec3> neutralTagPosition = new HashMap<>();

    private List<AprilTagDetection> lastDetections = Collections.emptyList();
    private VisionEstimate lastEstimate;
    private Pose lastAppliedPose;
    private long lastProcessedFrameNanoTime = Long.MIN_VALUE;
    private String lastStatus = "not updated";

    public BiobuzzVision(
            HardwareMap hardwareMap,
            String webcamName,
            CameraExtrinsics cameraExtrinsics
    ) {
        this(hardwareMap, webcamName, cameraExtrinsics, new Config());
    }

    public BiobuzzVision(
            HardwareMap hardwareMap,
            String webcamName,
            CameraExtrinsics cameraExtrinsics,
            Config config
    ) {
        this.config = config;
        this.cameraExtrinsics = cameraExtrinsics;

        // Pedro field is nominally 144 x 144 in. The official manual gives HIVE centers
        // 25.5 in apart and pivot axes 43.95 in above tiles.
        hivePivots.put(Hive.RED, new Vec3(72.0 - 12.75, 72.0, 43.95));
        hivePivots.put(Hive.BLUE, new Vec3(72.0 + 12.75, 72.0, 43.95));

        // Match-start orientation: red audience-side CELL up, blue scoring-side CELL up.
        hiveTracks.put(Hive.RED, new HiveTrack(Math.toRadians(-30.0)));
        hiveTracks.put(Hive.BLUE, new HiveTrack(Math.toRadians(30.0)));

        AprilTagLibrary library = buildBiobuzzTagLibrary();
        AprilTagProcessor.Builder processorBuilder = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawTagID(config.drawTagId)
                .setDrawTagOutline(config.drawTagOutline)
                .setDrawAxes(config.drawAxes)
                .setDrawCubeProjection(config.drawCube);

        if (config.fx > 0 && config.fy > 0 && config.cx > 0 && config.cy > 0) {
            processorBuilder.setLensIntrinsics(config.fx, config.fy, config.cx, config.cy);
        }

        aprilTag = processorBuilder.build();
        aprilTag.setDecimation(config.decimation);

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(config.cameraWidth, config.cameraHeight))
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
    }

    private static AprilTagLibrary buildBiobuzzTagLibrary() {
        AprilTagLibrary.Builder builder = new AprilTagLibrary.Builder();
        for (int id = FIRST_TAG_ID; id <= LAST_TAG_ID; id++) {
            builder.addTag(id, "BIOBUZZ " + id, TAG_SIZE_IN, DistanceUnit.INCH);
        }
        return builder.build();
    }

    // -------------------------------------------------------------------------
    // Primary update / Pedro integration
    // -------------------------------------------------------------------------

    /**
     * Process one NEW camera frame using Pedro's current odometry pose as a prior.
     * Returns null when there is no new usable camera frame.
     */
    public VisionEstimate update(Pose pedroPrior) {
        List<AprilTagDetection> detections = config.useFreshDetectionsOnly
                ? aprilTag.getFreshDetections()
                : aprilTag.getDetections();

        if (detections == null) {
            lastStatus = "no fresh frame";
            return null;
        }

        lastDetections = Collections.unmodifiableList(new ArrayList<>(detections));

        long frameTime = newestFrameTime(detections);
        if (frameTime > 0 && frameTime == lastProcessedFrameNanoTime) {
            lastStatus = "duplicate frame";
            return null;
        }
        if (frameTime > 0) lastProcessedFrameNanoTime = frameTime;

        List<AprilTagDetection> usable = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            if (isUsableDetection(detection) && hasTagGeometry(tagId(detection))) {
                usable.add(detection);
            }
        }

        if (usable.isEmpty()) {
            lastStatus = neutralTagPosition.isEmpty()
                    ? "no geometry configured"
                    : "no usable BIOBUZZ tags";
            lastEstimate = null;
            return null;
        }

        updateHiveAngle(Hive.RED, usable, pedroPrior.heading());
        updateHiveAngle(Hive.BLUE, usable, pedroPrior.heading());

        List<Observation> observations = new ArrayList<>();
        for (AprilTagDetection detection : usable) {
            Hive hive = hiveForTag(tagId(detection));
            if (hive == null || !hasFreshHiveAngle(hive)) continue;

            Vec3 currentTagField = getTagFieldPosition(tagId(detection), getHiveAngleRad(hive));
            Vec3 tagRobot = cameraToRobotTagVector(detection);
            if (currentTagField != null && tagRobot != null) {
                observations.add(new Observation(detection, tagRobot, currentTagField));
            }
        }

        if (observations.isEmpty()) {
            lastStatus = "no tag has a valid HIVE angle";
            lastEstimate = null;
            return null;
        }

        PoseFit fit = solvePoseWithOutlierRejection(observations, pedroPrior);
        if (fit == null) {
            lastStatus = "pose fit failed";
            lastEstimate = null;
            return null;
        }

        double avgRange = averageRange(fit.used);
        String gateReason = gateReason(pedroPrior, fit);
        boolean passes = gateReason == null;

        lastEstimate = new VisionEstimate(
                fit.pose,
                fit.used.size(),
                fit.headingFromVision,
                fit.rms,
                avgRange,
                frameTime,
                passes,
                passes ? "" : gateReason
        );
        lastStatus = passes ? "vision estimate accepted" : "vision estimate gated: " + gateReason;
        return lastEstimate;
    }

    /**
     * Process a fresh frame and fuse an accepted estimate into Pedro with follower.setPose().
     * Returns true only when a correction was actually applied.
     */
    public boolean updateAndApply(Follower follower) {
        VisionEstimate estimate = update(follower.pose());
        return estimate != null && applyEstimate(follower, estimate, false);
    }

    /** Same as updateAndApply(), but permits a caller-requested hard snap bypassing normal gates. */
    public boolean updateAndSnap(Follower follower) {
        VisionEstimate estimate = update(follower.pose());
        return estimate != null && applyEstimate(follower, estimate, true);
    }

    /**
     * Apply a previously computed estimate to Pedro.
     * forceSnap=true sets the full vision pose and ignores the odometry gate.
     */
    public boolean applyEstimate(Follower follower, VisionEstimate estimate, boolean forceSnap) {
        if (estimate == null) return false;
        if (!forceSnap && !estimate.passesGate) return false;

        Pose current = follower.pose();
        Pose applied;

        if (forceSnap) {
            applied = estimate.pose;
        } else {
            double rangeScale = rangeConfidenceScale(estimate.averageRangeIn);
            double posAlpha = (estimate.tagsUsed >= 2
                    ? config.multiTagPositionAlpha
                    : config.singleTagPositionAlpha) * rangeScale;
            posAlpha = clamp(posAlpha, 0.0, 1.0);

            double headingAlpha = estimate.headingFromVision
                    ? clamp(config.multiTagHeadingAlpha * rangeScale, 0.0, 1.0)
                    : 0.0;

            applied = blendPose(current, estimate.pose, posAlpha, headingAlpha);
        }

        follower.setPose(applied);
        lastAppliedPose = applied;
        lastStatus = forceSnap ? "vision snapped to Pedro" : "vision fused into Pedro";
        return true;
    }

    /** Apply last accepted estimate if available. Does not reprocess the camera. */
    public boolean applyLastEstimate(Follower follower) {
        return applyEstimate(follower, lastEstimate, false);
    }

    /** Hard-snap Pedro to the last estimate if available. Intended for explicit relocalization. */
    public boolean snapToLastEstimate(Follower follower) {
        return applyEstimate(follower, lastEstimate, true);
    }

    // -------------------------------------------------------------------------
    // Dynamic HIVE angle estimation
    // -------------------------------------------------------------------------

    private void updateHiveAngle(
            Hive hive,
            List<AprilTagDetection> detections,
            double pedroHeadingRad
    ) {
        List<AprilTagDetection> candidates = new ArrayList<>();
        for (AprilTagDetection d : detections) {
            if (hiveForTag(tagId(d)) == hive && hasTagGeometry(tagId(d)) && d.ftcPose != null) {
                candidates.add(d);
            }
        }
        if (candidates.isEmpty()) return;

        // Primary geometric fit from tag HEIGHT. Robot X/Y/heading do not enter this calculation.
        double heightAngle = searchHiveAngle(hive, candidates,
                config.minHiveAngleRad,
                config.maxHiveAngleRad,
                config.hiveAngleCoarseStepRad);

        double fineMin = Math.max(config.minHiveAngleRad, heightAngle - config.hiveAngleCoarseStepRad);
        double fineMax = Math.min(config.maxHiveAngleRad, heightAngle + config.hiveAngleCoarseStepRad);
        heightAngle = searchHiveAngle(hive, candidates, fineMin, fineMax, config.hiveAngleFineStepRad);

        double rms = hiveHeightRms(hive, candidates, heightAngle);
        if (!Double.isFinite(rms) || rms > config.maxHiveHeightResidualIn) return;

        // Your original insight: the tag PLANE itself rotates with the HIVE.
        // Estimate the same HIVE angle from the raw tag normal and use it as a cross-check/fusion.
        Double orientationAngle = config.useTagOrientationForHiveAngle
                ? estimateHiveAngleFromOrientations(candidates, pedroHeadingRad)
                : null;

        double measuredAngle = heightAngle;
        if (orientationAngle != null) {
            double disagreement = Math.abs(wrap(orientationAngle - heightAngle));
            if (disagreement <= config.maxOrientationHeightDisagreementRad) {
                double w = clamp(config.orientationAngleWeight, 0.0, 1.0);
                measuredAngle = wrap(heightAngle + w * wrap(orientationAngle - heightAngle));
            }
            // If the two disagree badly, keep the height solution. That avoids a bad raw
            // orientation estimate or a temporary Pedro-heading error corrupting localization.
        }

        HiveTrack track = hiveTracks.get(hive);
        if (track.valid) {
            double delta = wrap(measuredAngle - track.angleRad);
            track.angleRad = wrap(track.angleRad + config.hiveAngleSmoothing * delta);
        } else {
            track.angleRad = measuredAngle;
            track.valid = true;
        }
        track.angleRad = clamp(track.angleRad, config.minHiveAngleRad, config.maxHiveAngleRad);
        track.updateNanoTime = System.nanoTime();
        track.lastHeightRms = rms;
        track.tagsUsed = candidates.size();
    }

    /**
     * Estimate HIVE angle directly from one tag's plane normal. This is the orientation-based
     * version of the moving-tag idea: rawPose.R supplies the tag's orientation relative to the
     * camera; camera extrinsics and Pedro heading rotate that normal into field coordinates.
     *
     * Returns null if raw orientation is unavailable or geometrically inconsistent.
     */
    public Double estimateHiveAngleFromTagOrientation(
            AprilTagDetection detection,
            double pedroHeadingRad
    ) {
        if (detection == null || detection.rawPose == null || detection.rawPose.R == null) return null;

        // AprilTag raw camera coordinates -> FTC camera coordinates:
        // FTC x = raw x, FTC y = raw z, FTC z = -raw y.
        // The 3rd column of the tag->camera rotation is the tag-plane normal in raw camera axes.
        double rawNx = detection.rawPose.R.get(0, 2);
        double rawNy = detection.rawPose.R.get(1, 2);
        double rawNz = detection.rawPose.R.get(2, 2);
        Vec3 normalCamera = new Vec3(rawNx, rawNz, -rawNy);
        Vec3 normalRobot = cameraExtrinsics.cameraToRobot.multiply(normalCamera);

        double c = Math.cos(pedroHeadingRad);
        double s = Math.sin(pedroHeadingRad);
        Vec3 normalField = new Vec3(
                c * normalRobot.x - s * normalRobot.y,
                s * normalRobot.x + c * normalRobot.y,
                normalRobot.z
        );

        double norm = normalField.norm();
        if (norm < 1e-9) return null;
        normalField = normalField.times(1.0 / norm);

        // The HIVE tags are on the bottom face. Choose the equivalent normal that points downward.
        if (normalField.z > 0.0) normalField = normalField.times(-1.0);

        // For a perfect Rx-only HIVE rotation, normal X should remain ~0. Large X indicates
        // camera-extrinsic / Pedro-heading error or a poor orientation solve.
        if (Math.abs(normalField.x) > config.maxTagNormalXAbs) return null;

        // Neutral downward normal n0=(0,0,-1). Rx(phi)n0=(0,sin(phi),-cos(phi)).
        double angle = Math.atan2(normalField.y, -normalField.z);
        if (angle < config.minHiveAngleRad - Math.toRadians(5.0)
                || angle > config.maxHiveAngleRad + Math.toRadians(5.0)) {
            return null;
        }
        return clamp(angle, config.minHiveAngleRad, config.maxHiveAngleRad);
    }

    private Double estimateHiveAngleFromOrientations(
            List<AprilTagDetection> detections,
            double pedroHeadingRad
    ) {
        List<Double> angles = new ArrayList<>();
        for (AprilTagDetection d : detections) {
            Double a = estimateHiveAngleFromTagOrientation(d, pedroHeadingRad);
            if (a != null) angles.add(a);
        }
        if (angles.isEmpty()) return null;

        // Median is robust to one tag with a noisy corner/orientation solve.
        Collections.sort(angles);
        int n = angles.size();
        return n % 2 == 1
                ? angles.get(n / 2)
                : 0.5 * (angles.get(n / 2 - 1) + angles.get(n / 2));
    }

    private double searchHiveAngle(
            Hive hive,
            List<AprilTagDetection> detections,
            double min,
            double max,
            double step
    ) {
        HiveTrack track = hiveTracks.get(hive);
        double bestAngle = track.angleRad;
        double bestCost = Double.POSITIVE_INFINITY;

        for (double angle = min; angle <= max + 1e-9; angle += step) {
            double cost = 0.0;
            int count = 0;
            for (AprilTagDetection d : detections) {
                Vec3 robotPoint = cameraToRobotTagVector(d);
                Vec3 predictedField = getTagFieldPosition(tagId(d), angle);
                if (robotPoint == null || predictedField == null) continue;

                double residual = robotPoint.z - predictedField.z;
                // Huber-like cost so one bad pose solve does not dominate.
                double abs = Math.abs(residual);
                double k = 1.5;
                cost += abs <= k ? residual * residual : 2.0 * k * abs - k * k;
                count++;
            }
            if (count == 0) continue;
            cost /= count;

            // Tiny tie-breaker toward the previous estimate to avoid branch chatter.
            if (track.valid) {
                cost += 1e-4 * Math.pow(wrap(angle - track.angleRad), 2);
            }

            if (cost < bestCost) {
                bestCost = cost;
                bestAngle = angle;
            }
        }
        return bestAngle;
    }

    private double hiveHeightRms(Hive hive, List<AprilTagDetection> detections, double angleRad) {
        double sum = 0.0;
        int count = 0;
        for (AprilTagDetection d : detections) {
            if (hiveForTag(tagId(d)) != hive) continue;
            Vec3 robotPoint = cameraToRobotTagVector(d);
            Vec3 predicted = getTagFieldPosition(tagId(d), angleRad);
            if (robotPoint == null || predicted == null) continue;
            double e = robotPoint.z - predicted.z;
            sum += e * e;
            count++;
        }
        return count == 0 ? Double.POSITIVE_INFINITY : Math.sqrt(sum / count);
    }

    // -------------------------------------------------------------------------
    // Pose solving
    // -------------------------------------------------------------------------

    private PoseFit solvePoseWithOutlierRejection(List<Observation> observations, Pose prior) {
        List<Observation> working = new ArrayList<>(observations);
        PoseFit fit = solvePose(working, prior);
        if (fit == null) return null;

        // Remove one obviously bad tag at a time while enough observations remain.
        while (working.size() >= 3) {
            int worstIndex = -1;
            double worstResidual = -1.0;
            for (int i = 0; i < working.size(); i++) {
                double residual = xyResidual(working.get(i), fit.pose);
                if (residual > worstResidual) {
                    worstResidual = residual;
                    worstIndex = i;
                }
            }
            if (worstResidual <= config.maxPerTagResidualIn) break;
            working.remove(worstIndex);
            fit = solvePose(working, prior);
            if (fit == null) return null;
        }

        if (fit.rms > config.maxFitRmsIn) return null;
        return fit;
    }

    private PoseFit solvePose(List<Observation> observations, Pose prior) {
        if (observations.isEmpty()) return null;

        double qxBar = 0.0;
        double qyBar = 0.0;
        double pxBar = 0.0;
        double pyBar = 0.0;
        for (Observation o : observations) {
            qxBar += o.robotPoint.x;
            qyBar += o.robotPoint.y;
            pxBar += o.fieldPoint.x;
            pyBar += o.fieldPoint.y;
        }
        int n = observations.size();
        qxBar /= n;
        qyBar /= n;
        pxBar /= n;
        pyBar /= n;

        boolean canSolveHeading = n >= 2 && maxRobotPointBaseline(observations) >= config.minHeadingBaselineIn;
        double heading;

        if (canSolveHeading) {
            double dot = 0.0;
            double cross = 0.0;
            for (Observation o : observations) {
                double qx = o.robotPoint.x - qxBar;
                double qy = o.robotPoint.y - qyBar;
                double px = o.fieldPoint.x - pxBar;
                double py = o.fieldPoint.y - pyBar;
                dot += qx * px + qy * py;
                cross += qx * py - qy * px;
            }
            if (Math.hypot(dot, cross) < 1e-9) {
                canSolveHeading = false;
                heading = prior.heading();
            } else {
                heading = Math.atan2(cross, dot);
            }
        } else {
            heading = prior.heading();
        }

        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double x = pxBar - (c * qxBar - s * qyBar);
        double y = pyBar - (s * qxBar + c * qyBar);
        Pose pose = new Pose(x, y, wrap(heading));

        double sumSq = 0.0;
        for (Observation o : observations) {
            double r = xyResidual(o, pose);
            sumSq += r * r;
        }
        double rms = Math.sqrt(sumSq / n);
        return new PoseFit(pose, canSolveHeading, rms, new ArrayList<>(observations));
    }

    private static double xyResidual(Observation o, Pose pose) {
        double c = Math.cos(pose.heading());
        double s = Math.sin(pose.heading());
        double predictedX = pose.x() + c * o.robotPoint.x - s * o.robotPoint.y;
        double predictedY = pose.y() + s * o.robotPoint.x + c * o.robotPoint.y;
        return Math.hypot(predictedX - o.fieldPoint.x, predictedY - o.fieldPoint.y);
    }

    private static double maxRobotPointBaseline(List<Observation> observations) {
        double best = 0.0;
        for (int i = 0; i < observations.size(); i++) {
            for (int j = i + 1; j < observations.size(); j++) {
                Vec3 a = observations.get(i).robotPoint;
                Vec3 b = observations.get(j).robotPoint;
                best = Math.max(best, Math.hypot(a.x - b.x, a.y - b.y));
            }
        }
        return best;
    }

    private String gateReason(Pose prior, PoseFit fit) {
        double positionJump = Math.hypot(fit.pose.x() - prior.x(), fit.pose.y() - prior.y());
        if (positionJump > config.maxPositionCorrectionIn) {
            return String.format(Locale.US, "position jump %.1f in", positionJump);
        }
        if (fit.headingFromVision) {
            double headingJump = Math.abs(wrap(fit.pose.heading() - prior.heading()));
            if (headingJump > config.maxHeadingCorrectionRad) {
                return String.format(Locale.US, "heading jump %.1f deg", Math.toDegrees(headingJump));
            }
        }
        return null;
    }

    // -------------------------------------------------------------------------
    // Geometry configuration and calibration
    // -------------------------------------------------------------------------

    /** Override a fixed HIVE pivot in Pedro field coordinates. */
    public void setHivePivot(Hive hive, double x, double y, double z) {
        hivePivots.put(hive, new Vec3(x, y, z));
    }

    public Vec3 getHivePivot(Hive hive) {
        return hivePivots.get(hive);
    }

    /**
     * Register a tag's position relative to its HIVE pivot when HIVE angle = 0.
     * x/y/z are expressed in Pedro FIELD axes, in inches.
     */
    public void setTagNeutralPosition(int id, double x, double y, double z) {
        requireBiobuzzId(id);
        neutralTagPosition.put(id, new Vec3(x, y, z));
    }

    public void setTagNeutralPosition(int id, Vec3 neutralPosition) {
        setTagNeutralPosition(id, neutralPosition.x, neutralPosition.y, neutralPosition.z);
    }

    public void removeTagGeometry(int id) {
        neutralTagPosition.remove(id);
    }

    public void clearTagGeometry() {
        neutralTagPosition.clear();
    }

    public boolean hasTagGeometry(int id) {
        return neutralTagPosition.containsKey(id);
    }

    public int getConfiguredTagCount() {
        return neutralTagPosition.size();
    }

    public Vec3 getTagNeutralPosition(int id) {
        return neutralTagPosition.get(id);
    }

    public Map<Integer, Vec3> getAllTagNeutralPositions() {
        return Collections.unmodifiableMap(new LinkedHashMap<>(neutralTagPosition));
    }

    /** Current field position of a tag at a caller-specified HIVE angle. */
    public Vec3 getTagFieldPosition(int id, double hiveAngleRad) {
        Vec3 neutral = neutralTagPosition.get(id);
        Hive hive = hiveForTag(id);
        if (neutral == null || hive == null) return null;
        return hivePivots.get(hive).plus(rotateAboutX(neutral, hiveAngleRad));
    }

    /** Current field position using this class's tracked HIVE angle. */
    public Vec3 getTrackedTagFieldPosition(int id) {
        Hive hive = hiveForTag(id);
        if (hive == null || !hasFreshHiveAngle(hive)) return null;
        return getTagFieldPosition(id, getHiveAngleRad(hive));
    }

    /**
     * One-time geometry calibration helper.
     *
     * Preconditions:
     *  - Robot is stationary at an accurately known Pedro pose.
     *  - Target HIVE is stationary at the supplied accurately known angle.
     *  - Camera extrinsics are already accurate.
     *
     * Every currently visible tag on that HIVE gets a neutral vector computed and stored.
     * Repeat from another viewing position until all 16 tags are captured, then copy the output
     * from geometryAsJavaCode() into your permanent robot constants.
     */
    public CalibrationResult calibrateVisibleTags(
            Pose knownRobotPose,
            Hive hive,
            double knownHiveAngleRad
    ) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        List<Integer> calibrated = new ArrayList<>();
        Vec3 pivot = hivePivots.get(hive);

        for (AprilTagDetection d : detections) {
            if (hiveForTag(tagId(d)) != hive || !isUsableDetection(d)) continue;
            Vec3 robotPoint = cameraToRobotTagVector(d);
            if (robotPoint == null) continue;

            Vec3 fieldPoint = robotPointToField(robotPoint, knownRobotPose);
            Vec3 currentRelative = fieldPoint.minus(pivot);
            Vec3 neutral = rotateAboutX(currentRelative, -knownHiveAngleRad);
            neutralTagPosition.put(tagId(d), neutral);
            calibrated.add(tagId(d));
        }
        Collections.sort(calibrated);
        return new CalibrationResult(hive, calibrated.size(), calibrated);
    }

    /** Convenience stable-state calibration helper using +/-30 deg by default. */
    public CalibrationResult calibrateVisibleTagsAtStableState(
            Pose knownRobotPose,
            Hive hive,
            HiveState stableState
    ) {
        if (stableState != HiveState.AUDIENCE_UP && stableState != HiveState.SCORING_UP) {
            throw new IllegalArgumentException("stableState must be AUDIENCE_UP or SCORING_UP");
        }
        double angle = stableState == HiveState.SCORING_UP
                ? config.stableAngleRad
                : -config.stableAngleRad;
        return calibrateVisibleTags(knownRobotPose, hive, angle);
    }

    /**
     * Generates paste-ready Java lines for the currently calibrated geometry.
     */
    public String geometryAsJavaCode() {
        StringBuilder out = new StringBuilder();
        List<Integer> ids = new ArrayList<>(neutralTagPosition.keySet());
        Collections.sort(ids);
        for (int id : ids) {
            Vec3 p = neutralTagPosition.get(id);
            out.append(String.format(Locale.US,
                    "vision.setTagNeutralPosition(%d, %.6f, %.6f, %.6f);%n",
                    id, p.x, p.y, p.z));
        }
        return out.toString();
    }

    /**
     * Optional helper based on dimensions visible in the official manual.
     * It deliberately requires neutralZ from you rather than guessing it.
     *
     * Derived in-plane dimensions:
     *  - tag X offsets: +/-6.50 and +/-2.75 in from cluster center
     *  - cell front-to-tag-centerline: 9.938 - 2.75 = 7.188 in
     *  - overall HIVE length / 2: 42.91 / 2 = 21.455 in
     *  - therefore nominal tag centerline Y radius: 14.267 in
     *
     * Use CAD/calibration values instead of this helper when available.
     */
    public void configureManualPlanarGeometry(double neutralZ) {
        final double yRadius = 42.91 / 2.0 - (9.938 - 2.75); // 14.267 in

        // Scoring/opposite-audience CELL stickers are reversed left-to-right in field view.
        setTagNeutralPosition(30, +6.50, +yRadius, neutralZ);
        setTagNeutralPosition(31, +2.75, +yRadius, neutralZ);
        setTagNeutralPosition(32, -2.75, +yRadius, neutralZ);
        setTagNeutralPosition(33, -6.50, +yRadius, neutralZ);

        // Red audience CELL.
        setTagNeutralPosition(34, -6.50, -yRadius, neutralZ);
        setTagNeutralPosition(35, -2.75, -yRadius, neutralZ);
        setTagNeutralPosition(36, +2.75, -yRadius, neutralZ);
        setTagNeutralPosition(37, +6.50, -yRadius, neutralZ);

        // Blue audience CELL.
        setTagNeutralPosition(38, -6.50, -yRadius, neutralZ);
        setTagNeutralPosition(39, -2.75, -yRadius, neutralZ);
        setTagNeutralPosition(40, +2.75, -yRadius, neutralZ);
        setTagNeutralPosition(41, +6.50, -yRadius, neutralZ);

        // Blue scoring/opposite-audience CELL.
        setTagNeutralPosition(42, +6.50, +yRadius, neutralZ);
        setTagNeutralPosition(43, +2.75, +yRadius, neutralZ);
        setTagNeutralPosition(44, -2.75, +yRadius, neutralZ);
        setTagNeutralPosition(45, -6.50, +yRadius, neutralZ);
    }

    // -------------------------------------------------------------------------
    // HIVE state / tag lookup API
    // -------------------------------------------------------------------------

    public static boolean isBiobuzzTag(int id) {
        return id >= FIRST_TAG_ID && id <= LAST_TAG_ID;
    }

    public static Hive hiveForTag(int id) {
        if (id >= 30 && id <= 37) return Hive.RED;
        if (id >= 38 && id <= 45) return Hive.BLUE;
        return null;
    }

    public static CellSide cellSideForTag(int id) {
        if ((id >= 30 && id <= 33) || (id >= 42 && id <= 45)) return CellSide.SCORING;
        if ((id >= 34 && id <= 41)) return CellSide.AUDIENCE;
        return null;
    }

    public double getHiveAngleRad(Hive hive) {
        return hiveTracks.get(hive).angleRad;
    }

    public double getHiveAngleDeg(Hive hive) {
        return Math.toDegrees(getHiveAngleRad(hive));
    }

    public void seedHiveAngle(Hive hive, double angleRad) {
        HiveTrack track = hiveTracks.get(hive);
        track.angleRad = clamp(angleRad, config.minHiveAngleRad, config.maxHiveAngleRad);
        track.valid = false; // seed guides fit but is not considered a fresh visual measurement.
    }

    public void forceHiveAngle(Hive hive, double angleRad) {
        HiveTrack track = hiveTracks.get(hive);
        track.angleRad = clamp(angleRad, config.minHiveAngleRad, config.maxHiveAngleRad);
        track.valid = true;
        track.updateNanoTime = System.nanoTime();
    }

    public void invalidateHiveAngle(Hive hive) {
        HiveTrack track = hiveTracks.get(hive);
        track.valid = false;
        track.updateNanoTime = 0;
    }

    public boolean hasFreshHiveAngle(Hive hive) {
        HiveTrack track = hiveTracks.get(hive);
        if (!track.valid) return false;
        long ageMs = (System.nanoTime() - track.updateNanoTime) / 1_000_000L;
        return ageMs <= config.maxHiveAngleAgeMs;
    }

    public long getHiveAngleAgeMs(Hive hive) {
        HiveTrack track = hiveTracks.get(hive);
        if (!track.valid || track.updateNanoTime == 0) return Long.MAX_VALUE;
        return (System.nanoTime() - track.updateNanoTime) / 1_000_000L;
    }

    public double getHiveHeightRms(Hive hive) {
        return hiveTracks.get(hive).lastHeightRms;
    }

    public int getHiveAngleTagCount(Hive hive) {
        return hiveTracks.get(hive).tagsUsed;
    }

    public HiveState getHiveState(Hive hive) {
        if (!hasFreshHiveAngle(hive)) return HiveState.UNKNOWN;
        double angle = getHiveAngleRad(hive);
        if (Math.abs(angle - config.stableAngleRad) <= config.stableToleranceRad) {
            return HiveState.SCORING_UP;
        }
        if (Math.abs(angle + config.stableAngleRad) <= config.stableToleranceRad) {
            return HiveState.AUDIENCE_UP;
        }
        return HiveState.MOVING;
    }

    public boolean isHiveMoving(Hive hive) {
        return getHiveState(hive) == HiveState.MOVING;
    }

    public CellSide getUpCell(Hive hive) {
        HiveState state = getHiveState(hive);
        if (state == HiveState.SCORING_UP) return CellSide.SCORING;
        if (state == HiveState.AUDIENCE_UP) return CellSide.AUDIENCE;
        return null;
    }

    // -------------------------------------------------------------------------
    // Detection access / filtering
    // -------------------------------------------------------------------------

    public List<AprilTagDetection> getDetections() {
        return Collections.unmodifiableList(new ArrayList<>(aprilTag.getDetections()));
    }

    public List<AprilTagDetection> getLastProcessedDetections() {
        return lastDetections;
    }

    public List<AprilTagDetection> getUsableDetections() {
        List<AprilTagDetection> out = new ArrayList<>();
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (isUsableDetection(d)) out.add(d);
        }
        return Collections.unmodifiableList(out);
    }

    public AprilTagDetection getDetection(int id) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (tagId(d) != id) continue;
            if (best == null || decisionMargin(d) > decisionMargin(best)) best = d;
        }
        return best;
    }

    public boolean isTagVisible(int id) {
        return getDetection(id) != null;
    }

    public int getVisibleBiobuzzTagCount() {
        int count = 0;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (isBiobuzzTag(tagId(d))) count++;
        }
        return count;
    }

    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (isBiobuzzTag(tagId(d))) ids.add(tagId(d));
        }
        Collections.sort(ids);
        return Collections.unmodifiableList(ids);
    }

    public boolean isUsableDetection(AprilTagDetection d) {
        if (d == null || !isBiobuzzTag(tagId(d)) || d.ftcPose == null) return false;
        if (!finite(d.ftcPose.x) || !finite(d.ftcPose.y) || !finite(d.ftcPose.z)) return false;
        if (!finite(d.ftcPose.range)) return false;
        if (d.ftcPose.range < config.minRangeIn || d.ftcPose.range > config.maxRangeIn) return false;
        if (hamming(d) > config.maxHamming) return false;
        return decisionMargin(d) >= config.minDecisionMargin;
    }

    /** FTC camera-space tag center converted into robot coordinates. */
    public Vec3 cameraToRobotTagVector(AprilTagDetection d) {
        if (d == null || d.ftcPose == null) return null;
        Vec3 cameraVector = new Vec3(d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
        return cameraExtrinsics.translationRobot.plus(cameraExtrinsics.cameraToRobot.multiply(cameraVector));
    }

    /** Convert a robot-relative point to Pedro field coordinates using a known robot pose. */
    public static Vec3 robotPointToField(Vec3 robotPoint, Pose robotPose) {
        double c = Math.cos(robotPose.heading());
        double s = Math.sin(robotPose.heading());
        return new Vec3(
                robotPose.x() + c * robotPoint.x - s * robotPoint.y,
                robotPose.y() + s * robotPoint.x + c * robotPoint.y,
                robotPoint.z
        );
    }

    // -------------------------------------------------------------------------
    // Camera controls / lifecycle
    // -------------------------------------------------------------------------

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    public boolean isStreaming() {
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void setProcessorEnabled(boolean enabled) {
        visionPortal.setProcessorEnabled(aprilTag, enabled);
    }

    public boolean isProcessorEnabled() {
        return visionPortal.getProcessorEnabled(aprilTag);
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void setDecimation(float decimation) {
        aprilTag.setDecimation(decimation);
    }

    public void setCameraExtrinsics(CameraExtrinsics cameraExtrinsics) {
        this.cameraExtrinsics = cameraExtrinsics;
    }

    public CameraExtrinsics getCameraExtrinsics() {
        return cameraExtrinsics;
    }

    /** Non-blocking manual-exposure attempt. Returns false if the camera is not streaming. */
    public boolean trySetManualExposure(int exposureMs, int gain) {
        if (!isStreaming()) return false;
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            return true;
        } catch (RuntimeException ignored) {
            return false;
        }
    }

    public VisionEstimate getLastEstimate() {
        return lastEstimate;
    }

    public Pose getLastAppliedPose() {
        return lastAppliedPose;
    }

    public String getLastStatus() {
        return lastStatus;
    }

    public Config getConfig() {
        return config;
    }

    @Override
    public void close() {
        visionPortal.close();
    }

    // -------------------------------------------------------------------------
    // Telemetry / diagnostics
    // -------------------------------------------------------------------------

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Vision status", lastStatus);
        telemetry.addData("Camera", getCameraState());
        telemetry.addData("BIOBUZZ tags", getVisibleTagIds());
        telemetry.addData("Geometry", "%d/16 tags", getConfiguredTagCount());

        for (Hive hive : Hive.values()) {
            telemetry.addData(hive + " HIVE",
                    "%s  %.2f deg  age=%s  zRMS=%.2f  n=%d",
                    getHiveState(hive),
                    getHiveAngleDeg(hive),
                    getHiveAngleAgeMs(hive) == Long.MAX_VALUE ? "--" : getHiveAngleAgeMs(hive) + "ms",
                    getHiveHeightRms(hive),
                    getHiveAngleTagCount(hive));
        }

        if (lastEstimate != null) {
            telemetry.addData("Vision pose",
                    "(%.2f, %.2f, %.1fdeg)",
                    lastEstimate.pose.x(),
                    lastEstimate.pose.y(),
                    Math.toDegrees(lastEstimate.pose.heading()));
            telemetry.addData("Vision fit",
                    "n=%d rms=%.2f heading=%s gate=%s",
                    lastEstimate.tagsUsed,
                    lastEstimate.rmsResidualIn,
                    lastEstimate.headingFromVision,
                    lastEstimate.passesGate ? "PASS" : lastEstimate.rejectionReason);
        }
    }

    public String debugDetections() {
        StringBuilder out = new StringBuilder();
        for (AprilTagDetection d : aprilTag.getDetections()) {
            out.append(String.format(Locale.US,
                    "id=%d usable=%s range=%s margin=%.1f ham=%d ftc=(%s)%n",
                    tagId(d),
                    isUsableDetection(d),
                    d.ftcPose == null ? "--" : String.format(Locale.US, "%.1f", d.ftcPose.range),
                    decisionMargin(d),
                    hamming(d),
                    d.ftcPose == null ? "--" : String.format(Locale.US, "%.2f, %.2f, %.2f",
                            d.ftcPose.x, d.ftcPose.y, d.ftcPose.z)));
        }
        return out.toString();
    }

    // -------------------------------------------------------------------------
    // Math helpers
    // -------------------------------------------------------------------------

    private static final class Mat3 {
        final double[][] m;

        Mat3(double[][] m) {
            this.m = m;
        }

        static Mat3 rx(double a) {
            double c = Math.cos(a), s = Math.sin(a);
            return new Mat3(new double[][]{
                    {1, 0, 0},
                    {0, c, -s},
                    {0, s, c}
            });
        }

        static Mat3 ry(double a) {
            double c = Math.cos(a), s = Math.sin(a);
            return new Mat3(new double[][]{
                    {c, 0, s},
                    {0, 1, 0},
                    {-s, 0, c}
            });
        }

        static Mat3 rz(double a) {
            double c = Math.cos(a), s = Math.sin(a);
            return new Mat3(new double[][]{
                    {c, -s, 0},
                    {s, c, 0},
                    {0, 0, 1}
            });
        }

        Mat3 multiply(Mat3 other) {
            double[][] r = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 3; k++) {
                        r[i][j] += m[i][k] * other.m[k][j];
                    }
                }
            }
            return new Mat3(r);
        }

        Vec3 multiply(Vec3 v) {
            return new Vec3(
                    m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                    m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                    m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
            );
        }
    }

    private static Vec3 rotateAboutX(Vec3 v, double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Vec3(v.x, c * v.y - s * v.z, s * v.y + c * v.z);
    }

    private static Pose blendPose(Pose a, Pose b, double positionAlpha, double headingAlpha) {
        double x = lerp(a.x(), b.x(), positionAlpha);
        double y = lerp(a.y(), b.y(), positionAlpha);
        double dh = wrap(b.heading() - a.heading());
        double h = wrap(a.heading() + headingAlpha * dh);
        return new Pose(x, y, h);
    }

    private double rangeConfidenceScale(double rangeIn) {
        if (!Double.isFinite(rangeIn)) return config.minRangeScale;
        double normalized = 1.0 - (rangeIn - config.minRangeIn)
                / Math.max(1e-9, config.maxRangeIn - config.minRangeIn);
        return clamp(normalized, config.minRangeScale, 1.0);
    }

    private static double averageRange(List<Observation> observations) {
        double sum = 0.0;
        int count = 0;
        for (Observation o : observations) {
            if (o.detection.ftcPose != null && finite(o.detection.ftcPose.range)) {
                sum += o.detection.ftcPose.range;
                count++;
            }
        }
        return count == 0 ? Double.NaN : sum / count;
    }

    private static long newestFrameTime(List<AprilTagDetection> detections) {
        long newest = 0;
        for (AprilTagDetection d : detections) {
            newest = Math.max(newest, d.frameAcquisitionNanoTime);
        }
        return newest;
    }

    private static double decisionMargin(AprilTagDetection d) {
        return d instanceof AprilTagSingleDetection
                ? ((AprilTagSingleDetection) d).decisionMargin : Double.NEGATIVE_INFINITY;
    }

    // FTC 12 also returns cluster detections, which have no individual tag ID.
    private static int tagId(AprilTagDetection d) {
        return d instanceof AprilTagSingleDetection
                ? ((AprilTagSingleDetection) d).id : -1;
    }

    private static int hamming(AprilTagDetection d) {
        return d instanceof AprilTagSingleDetection
                ? ((AprilTagSingleDetection) d).hamming : Integer.MAX_VALUE;
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double wrap(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private static void requireBiobuzzId(int id) {
        if (!isBiobuzzTag(id)) {
            throw new IllegalArgumentException("BIOBUZZ HIVE tag ID must be 30..45, got " + id);
        }
    }
}
