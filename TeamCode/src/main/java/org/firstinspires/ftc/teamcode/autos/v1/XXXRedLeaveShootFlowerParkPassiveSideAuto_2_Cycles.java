////MeepMeep done
//
//
//
//
//package org.firstinspires.ftc.teamcode.autos.v1;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
//import org.firstinspires.ftc.teamcode.commands.CommandsV1;
//import org.firstinspires.ftc.teamcode.robot.subsystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.utils.Localizer;
//
//@Config
//@Autonomous(name = "Red Leave Shoot Flower Park Passive Side Auto 2 Cycles", group = "Autonomous")
//public class RedLeaveShootFlowerParkPassiveSideAuto_2_Cycles extends LinearOpMode {
//    protected CommandAbstract robot;
//
//    @Override
//    public void runOpMode() {
//        Pose2d initialPose = new Pose2d(60, 9.5, Math.toRadians(0));
//
//        robot = new CommandsV1(hardwareMap, initialPose);
//        robot.setIsBlue(false);
//
//        MecanumDrive md = robot.drivetrain;
//        Localizer localizer = md.getLocalizer();
//
//
//
//
//
//        TrajectoryActionBuilder tab2 = md.actionBuilder(initialPose)
//              // -10 x y 10
//                .strafeToLinearHeading(new Vector2d(47.0, 25), Math.toRadians(0));
//
//
//        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
//
//                .waitSeconds(1.0)
//
////                .strafeToLinearHeading(new Vector2d(-60, -9.5), Math.toRadians(179.9))
//                .strafeToLinearHeading(new Vector2d(56.0, 15.42), Math.toRadians(10.0))
//
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(50, 21.0), Math.toRadians(0));
//
//        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(47.0, 23.08), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(58, 9.5), Math.toRadians(0));
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            localizer.update();
//            Pose2d position = localizer.getPose();
//
//            telemetry.addData("X", position.position.x);
//            telemetry.addData("Y", position.position.y);
//            telemetry.addData("deg", Math.toDegrees(position.heading.toDouble()));
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//
//        Action trajectoryActionChosen2 = tab2.build();
//        Action trajectoryActionChosen3 = tab3.build();
//        Action trajectoryActionChosen4 = tab4.build();
//
//
//
//
//        runActionSafely(
//                new SequentialAction(
//                        robot.vision.checkForRedSideTag(),
//                        //robot.shooter.oldLaunchAction(),
//                        robot.intake.stopIntake(),
//
//                        new ParallelAction(
//
//                                trajectoryActionChosen2,
//                                robot.arm.armDownAction()
//                        ),
//                        robot.intake.spinUpIntake(),
//                        trajectoryActionChosen3,
//
//                        robot.arm.armUpAction(),
//                        trajectoryActionChosen4,
//                        robot.intake.stopIntake()
//
//                        //robot.intake.spinUpIntake()
//
//
//                        // if there is 25s left in the auto, just make a thing to make it park
//
//
//
//
//
//
//
//                ), 30.0, 99.0);
//
//
//
//    }
//
//    private void runActionSafely(Action action, double timeoutSeconds, double goHome) {
//        ElapsedTime timer = new ElapsedTime();
//        TelemetryPacket packet = new TelemetryPacket();
//        boolean goingHome = false;
//
//        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
//
//            if (!goingHome && timer.seconds() > goHome) {
//                goingHome = true;
//
//                robot.stopshoot();
//
//                robot.intake.stop();
//
//                MecanumDrive md = robot.drivetrain;
//                Localizer localizer = md.getLocalizer();
//                localizer.update();
//
//                action = robot.drivetrain
//                        .actionBuilder(localizer.getPose())
//                       //HOME
//                        .build();
//            }
//
//            if (!action.run(packet)) {
//                break;
//            }
//
//
//            telemetry.addData("Time", timer.seconds());
//            telemetry.update();
//            idle();
//        }
//
//
//
//
//
//        robot.stopshoot();
//        robot.setPower(0);
//        robot.intake.stop();
//
//        robot.drivetrain.setDrivePowers(
//                new PoseVelocity2d(
//                        new Vector2d(0, 0),
//                        0
//                )
//        );
//    }
//
//}
