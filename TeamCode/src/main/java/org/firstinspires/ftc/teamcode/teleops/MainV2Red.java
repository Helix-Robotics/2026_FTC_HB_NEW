package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.commands.CommandsV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.FeederV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Light;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterV2;

import java.util.List;

@TeleOp(name = "V2 Teleop")
public class MainV2Red extends MainV1Red {

    @Override
    public void init() {
        Intake intake = new Intake(hardwareMap);
        FeederV2 feederV2 = new FeederV2(hardwareMap);
        Light light = new Light(hardwareMap);
        ShooterV2 shooter = new ShooterV2(hardwareMap, feederV2, intake, light);
        robot = new CommandsV2(hardwareMap, new Pose2d(0, 0, 0)) ;
        stateMachine = StateMachine.WAITING_FOR_START;

        robot.feederDirection(true);
    }

    @Override
    public void loop() {
        // keep subsystems updated
        robot.update();
        robot.setintakePower(intakePower);
        if (gamepad2.rightBumperWasPressed())
        {
            if (!outtaking)
            {
                robot.feeder.reverseFeed();
                intaking = false;
                outtaking = true;
                intakePower = 1.0;
            }
            else
            {
                robot.feeder.stopfeed();
                outtaking = false;
                intakePower = 0.0;
            }
        }
        if (gamepad2.leftBumperWasPressed()) {
            if (!intaking) {
                robot.feeder.feed();

                intaking = true;
                outtaking = false;
                intakePower = -1.0;
            } else {
                robot.feeder.stopfeed();
                intaking = false;
                intakePower = 0.0;
            }
        }
        //intake.update(intakePower);

        /** Driver Operations **/
        // drivetrain
        bindCommonDriveTrain();
        List<Double> driveValues = bindDriveTrain();

        //binding
        /**Above is only for testing**/
        robot.fieldRelativeDrive(driveValues.get(1), driveValues.get(0), driveValues.get(2));


        //bindHood();


        ElapsedTime timer = new ElapsedTime();


        /** Below is hood bindings **/
        //Disab le manual feeding as ball may get stuck
        //And we will lose whole game
        //operator actually can stop the shooting cycle now


//        if (gamepad1.dpad_up) {robot.shooter.rpmUp();}
//        if (gamepad1.dpad_down) {robot.shooter.rpmDown();}
//        if (gamepad1.dpad_left) {robot.hoodUp5();}
//        if (gamepad1.dpad_right) {robot.hoodDown5();}

        //double distance = robot.getDistanceFromGoal(); just for now

        /**
         if (!robot.getIsBlue()){
         distance = distance+0.1;
         }
         **/

        double hoodTarget = 0;
        double rpmTarget = 0;

        /**  Limelight Aiming Seperate Logic Functions
         if(gamepad1.dpad_left &&  distance>0) { //This is for the close to tip shot
         double dCalc = 2.25; //we will cap at the distance we use
         if (distance < dCalc)
         dCalc = distance;
         hoodTarget = robot.calculateHoodAngle(dCalc);
         rpmTarget = robot.calculateShooterRPM(dCalc);

         robot.setHoodTarget(hoodTarget);
         robot.shooter.rpmSet(rpmTarget);
         robot.shooter.spinUpShooter();
         robot.aimAtTag(robot.getCameraTagX());
         }
         robot.turnToTag(gamepad1.dpadLeftWasPressed());

         if(gamepad1.dpad_right &&  distance>0) { //This is for the fence shot
         double dCalc = 2.25; //we will cap at the distance we use
         if (distance < dCalc)
         dCalc = distance;
         hoodTarget = robot.hood.calculateFencePositionToShoot(dCalc);
         rpmTarget = robot.shooter.calculateFenceRpmToShoot(dCalc);

         robot.setHoodTarget(hoodTarget);
         robot.shooter.rpmSet(rpmTarget);
         robot.shooter.spinUpShooter();
         robot.aimAtTag(robot.getCameraTagX());
         }
         robot.turnToTag(gamepad1.dpadRightWasPressed());
         **/

        //robot.turnToTag(gamepad1.dpadRightWasPressed()); // aims at april tag




        //if (gamepad2.dpad_left) {robot.stowHood();}

        /** SHOOTING STUFF **/
        //goal - close
        if (gamepad2.left_trigger == 1.0){

        }
        /** Current Shooting Spot is Against Goal, slightly a bit to the left **/



        //fence - mid
        //This is preset shooting


        if (gamepad1.aWasPressed()) {
            bindCommonDriveTrain();
        }

        if (gamepad2.right_trigger > 0.5) {
            robot.shoot(true, 5);
        }
        else if (gamepad1.dpad_down) {robot.reverseFeed();}
        else if (gamepad1.dpad_up) {robot.feed();}
        else {
            robot.stopshoot();
        }

        double shootvel = robot.shooter.getVelocity();

        ShooterV2.LaunchStateV2 shooterStatus = robot.shooter.get







        //close to tip - further away


        // This is limelight shooting
        if (gamepad2.right_bumper){

        }





        /**Testing for Driver Train**/
        /**Disabled**/
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

        //telemetryEssentials(telemetry, packet, distance);
        //telemetryAlignment(telemetry, packet);
        //telemetryEssentials(telemetry, packet, distance);
        //extraTelemtry(telemetry);
        /** End of Shooter Information **/
        //telemetryShooting(telemetry, packet);
        //telemetryHood(telemetry, packet, distance, hoodTarget, rpmTarget);
        //telemetryLocalisation(telemetry, packet);


        telementryRoadRunner(telemetry, packet);

        telemetry.addData("Shooter Velocity", shootvel);
        packet.put("Shooter Velocity", shootvel);

        telemetry.addData("Feeder", feeder.)

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        /**Start of Drive Train Information **/
        //telemetryDrivetrain(telemetry, packet);
        /**End of Drive Train Information**/

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }


}