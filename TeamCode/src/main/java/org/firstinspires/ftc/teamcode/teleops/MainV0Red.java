package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.CommandAbstract;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Main V0 Red (Don't Use!!!)")
public class MainV0Red extends OpMode {

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
    }

    StateMachine stateMachine ;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);

    protected CommandAbstract robot;

    double gyroRadians = 0;

    @Override
    public void init() {
        // tune inPerTick for ur drivetrain encoders

        //robot = new CommandsV0(hardwareMap, new Pose2d(0, 0, 0));
        //stateMachine = StateMachine.WAITING_FOR_START;
    }

    public void bindCommonDriveTrain(){
        if (gamepad1.a) {
            robot.drivetrain.resetImu();
        }

        /*if (gamepad1.b) {
            robot.turnToTag();
        }*/

        //if(gamepad1.x){
            //robot.drivetrain.setSlowMode(true);
        //}

        //if(gamepad1.y){
            //robot.drivetrain.setSlowMode(false);
        //}
    }

    /**Change this function for your different main function**/
    public List<Double> bindDriveTrain(){
        return bindRedDriveTrain();
    }

    public List<Double> bindRedDriveTrain(){
        double forward = gamepad1.left_stick_x;
        double right = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        List<Double> doubleList = new ArrayList<>();

        // Add Double values to the list
        doubleList.add(forward);
        doubleList.add(right);
        doubleList.add(turn);

        robot.setIsBlue(false);

        // Return the populated list
        return doubleList;
    }
    public List<Double> bindBlueDriveTrain(){
        double forward = -gamepad1.left_stick_x;
        double right = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        List<Double> doubleList = new ArrayList<>();

        // Add Double values to the list
        doubleList.add(forward);
        doubleList.add(right);
        doubleList.add(turn);

        robot.setIsBlue(true);

        // Return the populated list
        return doubleList;

    }
    public void bindShooter(){
        // shooter spin
        // Hold speed when trigger   is held
        //We should not do the spin
        /*if (gamepad2.right_trigger > 0.2){
            robot.shooter.setHold(true);
        }else if (gamepad2.left_trigger > 0.2){
            robot.shooter.setHold(false);
        }*/

        /**
        if (gamepad2.right_trigger > 0.2){
            robot.shooter.setBurstShotsRequested(2);
        }else if (gamepad2.left_trigger > 0.2){
            robot.shooter.setBurstShotsRequested(1);
        } **/

        /**
        if (gamepad2.y){
            robot.shooter.setHold(true);
            robot.shooter.spinUpShooter(1180);
        }
         **/

        /**if (gamepad2.right_bumper){
            robot.shooter.setBurstShotsRequested(3);
            //robot.shooter.setHold(true);
            //robot.shooter.spinUpShooter();
        } else if (gamepad2.left_bumper){
            robot.shooter.setHold(false);
            robot.shooter.stopShooter();
        } **/

        /**
        if (gamepad2.aWasPressed()) {
            robot.shooter.maxSpeed();
        }

        if (gamepad2.bWasPressed()) {
            robot.shooter.maxNegativeSpeed();
        }
         **/

        // Single-shot request on bumper press
        //robot.launch(gamepad1.rightBumperWasPressed());

        // Always update the shooter state machine each loop


        //if (gamepad2.xWasPressed()) {robot.shooter.updateShooterPID();}
    }

    public void bindHood(){

        //if (gamepad1.dpad_down) {robot.stowHood();}
        //if (gamepad1.dpad_left) {robot.setHoodPIDF();}
    }

    @Override
    public void loop() {
        // keep subsystems updated
        robot.update();

        /** Driver Operations **/
        // drivetrain
        bindCommonDriveTrain();
        List<Double> driveValues = bindDriveTrain();

        /**Above is only for testing**/
        robot.fieldRelativeDrive(driveValues.get(1), driveValues.get(0), driveValues.get(2));

        bindShooter();

        /**Testing for Driver Train**/
        /**We disable b binding**/
        /*
        switch (stateMachine) {
            case WAITING_FOR_START:
                if (gamepad1.b){
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                }
                break;

            case DRIVE_TO_TARGET_1:
                Pose2D pose2D = robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getPosition();
                if(robot.drivetrain.driveTo(pose2D, TARGET_1, 0.7, 0)){
                    telemetry.addLine("At Position 1 - ZERO");
                    stateMachine = StateMachine.AT_TARGET;
                }
                break;

            case AT_TARGET:
                telemetry.addData("Finish Position:", robot.getPodPose());
                break;

        }*/
        /**End of testing**/

        // telemetry
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //extraTelemtry(telemetry);
        /** End of Shooter Information **/
        //telemetryShooting(telemetry, packet);
        telemetryHood(telemetry, packet, 0, 0, 0);

        /**Start of Drive Train Information **/
        telemetryDrivetrain(telemetry, packet);
        /**End of Drive Train Information**/

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    protected void telemetryHood(Telemetry telemtry, TelemetryPacket packet, double distance, double hoodTarget, double rpmTarget){
        //do nothing for v0
    }

    /**
    public void telemetryShooting(Telemetry telemetry, TelemetryPacket packet){

        telemetry.addData("Distance to Tag", robot.getDistanceFromTag());
        packet.put("Distance to Tag", robot.getDistanceFromTag());
        telemetry.addData("Distance to Tag PODS", robot.getDistanceFromTagPODS());
        packet.put("Distance to Tag PODS", robot.getDistanceFromTagPODS());

        telemetry.addData("TagX", robot.getCameraTagX());
        packet.put("TagX", robot.getCameraTagX());
        telemetry.addData("TagY", robot.getCameraTagY());
        packet.put("TagY", robot.getCameraTagY());
    }
     **/

    public void telemetryDrivetrain(Telemetry telemtry, TelemetryPacket packet){
        telemetry.addData("FR Power", robot.getFRPower());
        packet.put("FR Power", robot.getFRPower());
        telemetry.addData("FL Power", robot.getFLPower());
        packet.put("FL Power", robot.getFLPower());
        telemetry.addData("BR Power", robot.getBRPower());
        packet.put("BR Power", robot.getBRPower());
        telemetry.addData("BL Power", robot.getBLPower());
        packet.put("BL Power", robot.getBLPower());
        telemetry.addData("GyroAngle(radians)", robot.drivetrain.getGyroRadians());
        Pose2d pPose = robot.getPodPose();
        //please note for the pod, x in the direction of robt, y in the direction of per to robot
        //telemetry.addData("Robot Pod POS", pPose.toString());
        telemetry.addData("Robot Pos X (mm):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Pos Y (mm):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Heading (degrees):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getHeading(AngleUnit.DEGREES));

        //packet.put("Robot Pod POS", pPose.toString());
        packet.put("Robot Pos X (mm):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getPosX(DistanceUnit.MM));
        packet.put("Robot Pos Y (mm):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getPosY(DistanceUnit.MM));
        packet.put("Robot Heading (degrees):", robot.drivetrain.getHelixLocalizer().getLocalizer().driver.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Robot Slow Mode", robot.drivetrain.getSlowMode());
        packet.put("Robot Slow Mode", robot.drivetrain.getSlowMode());
        /*
        Pose2d vPose = robot.getVisionPose();
        if (vPose == null){
            telemetry.addData("Robot Vision POS", "null");
        } else {
            telemetry.addData("Robot Vision POS", vPose.toString());
        }
        */
    }
}
