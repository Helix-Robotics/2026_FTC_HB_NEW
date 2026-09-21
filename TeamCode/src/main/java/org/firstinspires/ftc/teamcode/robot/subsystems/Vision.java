package org.firstinspires.ftc.teamcode.robot.subsystems;//package org.firstinspires.ftc.teamcode.robot.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import java.util.List;
//import java.util.Map;
//
//
///**
// * Vision class for AprilTag-based localization using the Limelight 3A.
// */
//@Config
//public class Vision {
//
//    private final Limelight3A limelight;
//    private LLResult latestResult;
//
//    public static int middlePixel = 320;
//
//    /**
//     * 1 is blue and 2 is red
//     */
//    private final Map<Integer, Vector2d> aprilTagFieldPositions = Map.of(
//            20, new Vector2d(-1.482, -1.413),   // Example position for Tag 1
//            24, new Vector2d(-1.482, 1.413)    // etc.
//            // Add all known tags here based on your field layout
//    );
//
//    public Vision(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.start();
//        limelight.pipelineSwitch(0); // Set to AprilTag pipeline
//    }
//
//    /**
//     * Updates the Limelight's internal orientation (usually from IMU).
//     * @param headingYawDeg Robot heading from IMU (in degrees).
//     */
//    public void updateRobotOrientation(double headingYawDeg) {
//        limelight.updateRobotOrientation(headingYawDeg);
//    }
//
//    /**
//     * Call this method every loop to refresh Limelight data.
//     */
//    public void update() {
//        latestResult = limelight.getLatestResult();
//    }
//
//    /**
//     * Checks if a valid AprilTag pose has been detected.
//     */
//    public boolean hasValidPose() {
//        return latestResult != null && latestResult.isValid() && latestResult.getBotpose_MT2() != null;
//    }
//
//    /**
//     * Returns the current robot pose estimated by the AprilTag (as Pose2d).
//     * X/Y in inches, heading in radians.
//     */
//    public Pose2d getBotPose() {
//        if (!hasValidPose()) return null;
//
//        Pose3D botpose = latestResult.getBotpose_MT2();
//
//        double x = botpose.getPosition().x; // meters
//        double y = botpose.getPosition().y; // meters
//        double headingRad = Math.toRadians(botpose.getOrientation().getYaw()); //radian
//
//        return new Pose2d(x, y, headingRad);
//    }
//
//    /**
//     * Returns the 3D Euclidean distance from the robot to the AprilTag (in inches).
//     * I DONT THINK THIS WORKS.
//     */
//    /**
//     * Returns the horizontal (2D) distance from the robot to the detected AprilTag,
//     * using known tag field positions.
//     */
//    public double getDistanceToTagOnField() {
//        if (!hasValidPose()) return -1;
//
//        Pose3D botpose = latestResult.getBotpose_MT2();
//        List<LLResultTypes.FiducialResult> tagId = latestResult.getFiducialResults();
//        int tagIdNumber = tagId.get(0).getFiducialId();
//
//        Vector2d tagPos = aprilTagFieldPositions.get(tagIdNumber);
//        if (tagPos == null) return -1;
//
//        double robotX = botpose.getPosition().x;
//        double robotY = botpose.getPosition().y;
//
//        double dx = tagPos.x - robotX;
//        double dy = tagPos.y - robotY;
//
//        return Math.hypot(dx, dy);
//
//    }
//
//    /**
//     * Returns the latency of the Limelight result (in ms).
//     */
//    public double getLatencyMs() {
//        return latestResult != null ? latestResult.getCaptureLatency()+latestResult.getParseLatency() : 0.0;
//    }
//
//    /**
//     * Gets raw LLResult (optional for debugging).
//     */
//    public LLResult getLatestResult() {
//        return latestResult;
//    }
//
//    public double getTagX() {
//        if (latestResult.isValid()) {
//            return latestResult.getTx();
//        } else {
//            //we just assume we wont have -181 degree off tx
//            return -181;
//        }
//    }
//
//    public double getTagY() {return latestResult.getTy();}
//    public double getTa(){return latestResult.getTa();}
//}
