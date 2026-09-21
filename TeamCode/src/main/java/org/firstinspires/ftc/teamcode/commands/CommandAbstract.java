package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.robot.subsystems.HelixLocalisation;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Localizer;

public abstract class CommandAbstract {
    private HelixLocalisation helixLocaliser;
    private Localizer localiser;
    // protected Vision vision; just for now
    public MecanumDrive drivetrain;
    public Intake intake;
    public Feeder feeder;
    public Shooter shooter;


    //private LedController ledController;


    protected HardwareMap hardwareMap;
    protected boolean isAiming = false;
    protected boolean isBlue = false;
    public static double TX_TARGET_BLUE = -0.5;   // This is for close tip shots  //-0.8; //-5.12;
    public static double TX_TARGET_RED = 0.8; // This is for close tip shots  //3.4 //before 16 may is 0.8  //1.0; //2.8; //2.98;

    public static double TX_TARGET_RED_Special = 0.5; // This is for auto  // 3.9 furthest to the right position for red in the teleop
    public static double TX_TARGET_BLUE_Special = -2.0; // This is for auto

    public static double TX_TOLERANCE = 0.1; // 0.08
    public static double ALIGN_KP = 0.01; // 0.01 is old value
    public static double ALIGN_KF = 0.11; //0.275 //0.11

    public static double ALIGN_KP_Special = 0.005; //auto kp

    public static double ALIGN_KF_Special = 0.083; //auto kf

    public static double ALIGN_CAP_POWER = 0.4;

    //double distance = getDistanceFromGoal(); my changes for the variable tx

    public enum ALIGN_STATUS {
        PENDING,
        START,
        IN_PROGRESS,
        FINISH
    }

    public static ALIGN_STATUS alignStatus = ALIGN_STATUS.PENDING;

    public CommandAbstract(HardwareMap hardwareMap, Pose2d initialPose) {
        this.hardwareMap = hardwareMap;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drivetrain = new MecanumDrive(hardwareMap, startPose);
        helixLocaliser = drivetrain.getHelixLocalizer();
        localiser = drivetrain.getLocalizer();
        intake = new Intake(hardwareMap);
        feeder = new Feeder(hardwareMap);
        shooter = new Shooter(hardwareMap);

        //vision = drivetrain.getVision(); just for now

        //ledController = new LedController(hardwareMap);

        createShooterInstances();
    }

    public void setIsBlue(boolean isBlue){
        this.isBlue = isBlue;
    }

    public abstract void createShooterInstances();



    // run every loop
    public void update() {
        helixLocaliser.updateLocalisation();
        drivetrain.update();
        //intake.update();
    }

    public void shoot(double targetVel) {
        shooter.shoot(targetVel);
    }

    public void stopshoot() {
        shooter.stop();
    }

    public void setFeeder(double power) {
        feeder.setFeeder(power);
    }

    public void setintakePower(double intakepower){
        intake.setPower(intakepower);
    }

//    public void setintake(double power) {intake.setintake(power);}

    // get launch state









    public void fieldRelativeDrive(double right, double forward, double rotate) {
        drivetrain.driveFieldRelative(forward, right, rotate);
    }

    public void aimAtTag(double tagX) {
        //Dont use this
        //drivetrain.drive(0, 0, (tagX)*-0.2);
    }

    public void alignToTag() {
        /*double tagX = vision.getTagX();
        if (tagX > 1 || tagX < -1) {
            drivetrain.drive(0, 0, tagX * 0.02);
        }*/
        //Dont use this
    }

    public ALIGN_STATUS getAlignStatus(){
        return alignStatus;
    }

    /**
    public boolean turnToTagO(){
        //hard code pid just for turning
        double tagX = vision.getTagX();
        double txTarget = TX_TARGET_RED;
        double txTolerance = TX_TOLERANCE;
        double kP = ALIGN_KP;
        double kF = ALIGN_KF;
        double maxTurnPower = ALIGN_CAP_POWER;

        //decide which target to use
        if (isBlue){
            txTarget = TX_TARGET_BLUE;
        }

        // invalid result
        if(tagX < -180) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        double error = tagX - txTarget;
        //if under tolerance, return
        if(Math.abs(error) < txTolerance) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        //calculate turn power using P and F , we wont use D as it will be small angle
        //we can just use small P
        double turnPower = kP*error + kF*error/Math.abs(error);
        if (Math.abs(turnPower) > maxTurnPower) {
            turnPower = maxTurnPower * turnPower / Math.abs(turnPower);
        }
        drivetrain.drive(0, 0, turnPower);
        return false;
    }

    public void turnToTag(boolean startTurn){
        switch(alignStatus){
            case PENDING:
                if (startTurn){
                    alignStatus = ALIGN_STATUS.START;
                }
                break;
            case START:
            case IN_PROGRESS:
                boolean finished = turnToTagO();
                if (finished){
                    alignStatus = ALIGN_STATUS.FINISH;
                } else {
                    alignStatus = ALIGN_STATUS.IN_PROGRESS;
                }
                break;
            case FINISH:
                drivetrain.drive(0, 0, 0);
                alignStatus = ALIGN_STATUS.PENDING;
                break;
        }
    }



    public void turnToTagLongShooting(boolean startTurn){
        switch(alignStatus){
            case PENDING:
                if (startTurn){
                    alignStatus = ALIGN_STATUS.START;
                }
                break;
            case START:
            case IN_PROGRESS:
                boolean finished = turnToTagLongShootingO();
                if (finished){
                    alignStatus = ALIGN_STATUS.FINISH;
                } else {
                    alignStatus = ALIGN_STATUS.IN_PROGRESS;
                }
                break;
            case FINISH:
                drivetrain.drive(0, 0, 0);
                alignStatus = ALIGN_STATUS.PENDING;
                break;
        }
    }


    public boolean turnToTagLongShootingO(){
        //hard code pid just for turning
        double tagX = vision.getTagX();
        double txTarget = TX_TARGET_RED_Special;
        double txTolerance = TX_TOLERANCE;
        double kP = ALIGN_KP_Special;
        double kF = ALIGN_KF_Special;
        double maxTurnPower = ALIGN_CAP_POWER;

        //decide which target to use
        if (isBlue){
            txTarget = TX_TARGET_BLUE_Special;
        }

        // invalid result
        if(tagX < -180) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        double error = tagX - txTarget;
        //if under tolerance, return
        if(Math.abs(error) < txTolerance) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        //calculate turn power using P and F , we wont use D as it will be small angle
        //we can just use small P
        double turnPower = kP*error + kF*error/Math.abs(error);
        if (Math.abs(turnPower) > maxTurnPower) {
            turnPower = maxTurnPower * turnPower / Math.abs(turnPower);
        }
        drivetrain.drive(0, 0, turnPower);
        return false;
    }

    public class TurnToTagLongShootingActions implements Action {

        //private boolean hold = false;


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            //hood.setTargetPosition(220);
            vision.update();
            boolean finished = turnToTagLongShootingO();
            if (finished) {
                drivetrain.drive(0, 0, 0);
                return false;
            }
            return true;

        }
    }

    public Action turnToTagLongShootingActions() {
        return new TurnToTagLongShootingActions();
    }

     **/



    public void turnToTagVariableFarShooting(boolean startTurn){
        switch(alignStatus){
            case PENDING:
                if (startTurn){
                    alignStatus = ALIGN_STATUS.START;
                }
                break;
            case START:
            case IN_PROGRESS:
                boolean finished = turnToTagVariableFarShootingO();
                if (finished){
                    alignStatus = ALIGN_STATUS.FINISH;
                } else {
                    alignStatus = ALIGN_STATUS.IN_PROGRESS;
                }
                break;
            case FINISH:
                drivetrain.drive(0, 0, 0);
                alignStatus = ALIGN_STATUS.PENDING;
                break;
        }
    }


    public boolean turnToTagVariableFarShootingO(){
        /**

     
        double txTarget = TX_TARGET_RED_Special;
        //double txTarget = shooter.variableTXCalc(distance);
        double txTolerance = TX_TOLERANCE;
        double kP = ALIGN_KP_Special;
        double kF = ALIGN_KF_Special;
        double maxTurnPower = ALIGN_CAP_POWER;

        //decide which target to use
        if (isBlue){
            txTarget = TX_TARGET_BLUE_Special;
        }

        // invalid result
        if(tagX < -180) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        double error = tagX - txTarget;
        //if under tolerance, return
        if(Math.abs(error) < txTolerance) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        //calculate turn power using P and F , we wont use D as it will be small angle
        //we can just use small P
        double turnPower = kP*error + kF*error/Math.abs(error);
        if (Math.abs(turnPower) > maxTurnPower) {
            turnPower = maxTurnPower * turnPower / Math.abs(turnPower);
        }
        drivetrain.drive(0, 0, turnPower);
        return false;
         **/

        //double distance = getDistanceFromGoal();  just for now

        //double tagX = vision.getTagX(); just for now

        // Calculate the target using your regression formula instead of a hardcoded value


        double txTolerance = TX_TOLERANCE;
        double kP = ALIGN_KP_Special;
        double kF = ALIGN_KF_Special;
        double maxTurnPower = ALIGN_CAP_POWER;


        /**
        // Decide which target to use for Blue Alliance
        if (isBlue){
            // Note: Depending on your tuning, you might want this to be negative
            // e.g., txTarget = -shooter.variableTXCalc(distance);
            txTarget = TX_TARGET_BLUE_Special;
        }
         **/

        // invalid result
        //if(tagX < -180) {
          //  drivetrain.drive(0, 0, 0);    just for now
            //return true;
        //}

        /**
        //double txTarget = shooter.variableTXCalc(distance);
        //double error = tagX - txTarget;

        // if under tolerance, return
        if(Math.abs(error) < txTolerance) {
            drivetrain.drive(0, 0, 0);
            return true;
        }

        // calculate turn power using P and F
        double turnPower = kP*error + kF*error/Math.abs(error);
        if (Math.abs(turnPower) > maxTurnPower) {
            turnPower = maxTurnPower * turnPower / Math.abs(turnPower);
        }

        drivetrain.drive(0, 0, turnPower);
        return false;
         **/
        return false;
    }




    public Pose2d getPodPose(){
        return helixLocaliser.getPose();
    }


    /**   Commented out vision stuff just for now
    public Vision getVision(){
        return vision;
    }

    public Pose2d getVisionPose() {
        return vision.getBotPose();
    }

    public double getCameraTagX(){
        return vision.getTagX();
    }
    public double getCameraTagY(){
        return vision.getTagY();
    }
    public double getCameraTagTa(){
        return vision.getTa();
    }
     **/




    public double getFRPower() { return drivetrain.getFRPower(); }
    public double getFLPower() { return drivetrain.getFLPower(); }
    public double getBRPower() { return drivetrain.getBRPower(); }
    public double getBLPower() { return drivetrain.getBLPower(); }

    //public double getDistanceFromTag() { return vision.getDistanceToTagOnField(); } just for now
    public double getDistanceFromTagPODS() {
        return helixLocaliser.getDistanceFromGoal(isBlue);
    }

    public double getDistanceFromGoalFused() { return helixLocaliser.getDistanceFromGoal(isBlue);}

    //public double getDistanceFromGoal() {
      //  return getDistanceFromTag();  just  for now
    //}

    public void resetImu() {
        drivetrain.resetImu();
    }





    public boolean getIsBlue(){
        return isBlue;
    }



}
