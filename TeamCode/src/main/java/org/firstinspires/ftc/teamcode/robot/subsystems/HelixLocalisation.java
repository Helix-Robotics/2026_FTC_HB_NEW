package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PinpointLocalizer;

public final class HelixLocalisation {
    private static HelixLocalisation helixLocalisation = null;
    public static PinpointLocalizer localizer; //used to be public for the telementry
    //private static Vision vision;  just for now

    private String fusedDateSource = "xxx Waiting";

    private Pose2d lastVisVisionPose2d = new Pose2d(0, 0, 0);
    private Pose2d lastVisOdoPose2d = new Pose2d(0, 0, 0);
    private boolean lastVisInitilised = false;

    /*public static synchronized HelixLocalisation getInstance(HardwareMap hardwareMap, double inPerTick, Pose2d initialPos){
        if (helixLocalisation == null)
            helixLocalisation = new HelixLocalisation(hardwareMap, inPerTick, initialPos);

        //reset pinpoint localiser if required
        localizer.setPose(initialPos);

        return helixLocalisation;
    }*/

    public HelixLocalisation(HardwareMap hardwareMap, double inPerTick, Pose2d initialPos) {
        //vision = new Vision(hardwareMap); just for now
        Pose2d botPos = null;
        //botPos = vision.getBotPose(); just for now
        /**We will use camera botpos to initalise position**/
        /**It seems that it doesn't work**/
        if (botPos != null){
            initialPos = botPos;
        }
        localizer = new PinpointLocalizer(hardwareMap, inPerTick, initialPos);
    }

    //public double getVisionDistanceFromGoal(){
        //return vision.getDistanceToTagOnField();    this function just for now commented
    //}

    public double getDistanceFromGoal(boolean isBlue) {
        Pose2d robotPos = getFusedPos();

        if (robotPos == null){
            return -1;
        }

        double tagposy;
        double tagposx = -1.482;
        if (isBlue) {
            tagposy = -1.413;
        }
        else {tagposy = 1.413;}

        return Math.hypot(robotPos.position.x-tagposx, robotPos.position.y-tagposy);
    }

    public PinpointLocalizer getLocalizer(){
        return localizer;
    }

    //public Vision getVision(){
        //return vision;    just for now
    //}

    public void updateLocalisation() {
        /**We disable botpos update from vision for now**/
        /**
        Pose2d botPos = vision.getBotPose();
        if (botPos != null){
            localizer.setPose(botPos);
        }**/
        localizer.update();
        //vision.update(); just for now

        //Pose2d pose = getVisionPos(); just for now

        //if (pose != null){
          //  lastVisInitilised = true;
            //lastVisVisionPose2d = pose;    just for now full function
            //lastVisOdoPose2d = getOdoPose();
        //}
    }

    public static Pose2d getOdoPose() { return localizer.getPose(); }
    //public static Pose2d getVisionPos() { return vision.getBotPose(); } just for now

    public String getFusedDateSource(){
        return fusedDateSource;
    }
    public Pose2d getFusedPos() {

        //Pose2d visionPos = getVisionPos(); //in meters and radians just for now
        Pose2d odoPos = getOdoPose(); // in inches and radians

        //if we see vison pos, we will return vision bot pos
        //if(visionPos != null){
          //  fusedDateSource = "--- Vision";   //this function just for now
            //return visionPos;
        //}

        if (lastVisInitilised == false){
            return null;
        }

        //caculation pos = lastVisionPos + (currentOdoPos - lastOdoPos)
        /** We do some unit coversion here **/
        double x = lastVisVisionPose2d.position.x + (odoPos.position.x - lastVisOdoPose2d.position.x)*0.0254;
        double y = lastVisVisionPose2d.position.y + (odoPos.position.y - lastVisOdoPose2d.position.y)*0.0254;
        double heading = lastVisVisionPose2d.heading.real + (odoPos.heading.real - lastVisOdoPose2d.heading.real);

        fusedDateSource = "*-*-* calculated";
        return new Pose2d(x,y,heading);
    }

    public Pose2d getPose() {
        return localizer.getPose();
    }

    public Pose2d getLastVisVisionPose2d(){
        return lastVisVisionPose2d;
    }

    public Pose2d getLastVisOdoPose2d(){
        return lastVisOdoPose2d;
    }
}
