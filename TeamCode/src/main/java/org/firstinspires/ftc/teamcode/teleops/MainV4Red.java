package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.CommandAbstract;
import org.firstinspires.ftc.teamcode.commands.CommandsV4;
import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterAbstract.LaunchState;

import java.util.List;

@Config
@TeleOp(name = "V4 Red Teleop")
public class MainV4Red extends MainV3Red {

    public static double headingP = 0.02;
    public static double headingD = 0.003;
    public static double maxTurnPower = 0.25;
    public static double turnStickDeadband = 0.08;
    public static double maxShotTurnDegrees = 2;
    public static double hiveCornerDifferenceInches = 3;

    public static double hiveLeftAimOffsetDegrees = 2.5; // pos aim right
    public static double hiveRightAimOffsetDegrees = -2.5;

    private double heldHeading = Double.NaN;
    private double shotStartHeading = Double.NaN;
    private boolean shootTriggerWasHeld;
    private double lastTagFrame = Double.NaN;
    private long lastTagFrameMs;
    private boolean hiveLock;
    private boolean hiveTriggerWasHeld;
    private double hiveHeading = Double.NaN;
    private double lastHiveFrame = Double.NaN;
    private boolean hasHiveTarget;
    private HiveTarget.AimPoint hiveAimPoint = HiveTarget.AimPoint.CENTER;

    private boolean slowMode = false;

    private boolean holdIntake = false;


    @Override
    public void init() {
        robot = new CommandsV4(hardwareMap, new Pose2d(0, 0, 0));
        CommandAbstract.alignStatus = CommandAbstract.ALIGN_STATUS.PENDING;
        heldHeading = Double.NaN;
        shotStartHeading = Double.NaN;
        shootTriggerWasHeld = false;
        lastTagFrame = Double.NaN;
        hiveLock = false;
        hiveTriggerWasHeld = false;
        hiveHeading = Double.NaN;
        lastHiveFrame = Double.NaN;
        hasHiveTarget = false;
        hiveAimPoint = HiveTarget.AimPoint.CENTER;

        robot.drivetrain.setSlowMode(false);

        slowMode = false;

        holdIntake = false;

        robot.feederDirection(false);
        robot.intakeDirection(true);
        //robot.shooterDirection(true);
        robot.shooter.setHold(true);

        stateMachine = StateMachine.WAITING_FOR_START;
    }

    @Override
    public void loop() {
        // keep subsystems updated
        robot.update();
        robot.feeder.update();

        /** Driver Operations **/
        // drivetrain
        bindCommonDriveTrain();
        List<Double> driveValues = bindDriveTrain();
        double turnStick = driveValues.get(2);

        boolean shotInProgress = robot.shooter.getLaunchState() != LaunchState.IDLE;
        boolean hiveTriggerHeld = gamepad1.left_trigger > 0.5;
        if (hiveTriggerHeld && !hiveTriggerWasHeld && !shotInProgress) {
            hiveLock = !hiveLock;
            hiveAimPoint = HiveTarget.AimPoint.CENTER;
        }
        hiveTriggerWasHeld = hiveTriggerHeld;
        boolean manualTurn = !hiveLock && Math.abs(turnStick) > turnStickDeadband;
        double heading = Math.toDegrees(robot.drivetrain.getGyroRadians());
        double turnRate = robot.drivetrain.getIMU().getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

        if (!hiveLock || !Double.isFinite(heading)) {
            heldHeading = Double.NaN;
        } else if (gamepad1.a || !Double.isFinite(heldHeading)) {
            heldHeading = heading;
        }


        boolean shootTriggerHeld = gamepad2.left_trigger > 0.75 || gamepad2.right_trigger > 0.75;
        boolean shootPressed = shootTriggerHeld && !shootTriggerWasHeld;
        shootTriggerWasHeld = shootTriggerHeld;
        if (shootPressed && !shotInProgress) {
            shotStartHeading = heading;
        }
        double shotHeadingChange = Math.abs(AngleUnit.normalizeDegrees(heading - shotStartHeading));
        boolean shotHeadingOkay = Double.isFinite(shotHeadingChange) && shotHeadingChange <= maxShotTurnDegrees;
        LLResult frame = robot.vision.getLatestResult();
        boolean freshFrame = isFreshTagFrame(frame);
        updateHiveAim(frame, freshFrame, !shotInProgress && !shootPressed, heading);
        boolean blue = robot.getIsBlue();
        boolean hasTarget = hiveLock ? hasHiveTarget
                : freshFrame && robot.getFilteredCameraTagX(blue ? 41 : 33, blue ? 42 : 34) > -180;
        boolean missingStartTarget = !shotInProgress && !hasTarget;

        /** Intake Related **/
        if (gamepad2.rightBumperWasPressed())
        {
            if (!robot.intake.checkOut())
            {
                robot.intake.out();
                robot.feeder.reverseFeed();
            }
            else
            {
                robot.intake.stop();
                robot.feeder.stopfeed();

            }
        }






        if (gamepad2.leftBumperWasPressed()) {
            robot.light.blue();
            if (!robot.intake.checkIn())
            {
                robot.intake.in();
                robot.feeder.intakeSlowFeed();
            }
            else {
                robot.intake.stop();
                robot.feeder.stopfeed();
            }
        }


        if (gamepad2.dpadUpWasPressed()) {

            if (!robot.intake.holdIntakeCheck()) {
                robot.intake.hold();
                robot.feeder.stopfeed();
            } else {
                robot.intake.stop();
                robot.feeder.stopfeed();
            }
        }







        if (gamepad2.aWasPressed() || gamepad2.bWasPressed()) {
            robot.stopshoot();
        }

        if (gamepad2.a) {  // this unjam feeder
            robot.light.pink();
            robot.intake.unjam();
            robot.feeder.reverseFeed();

        } else if (gamepad2.b) { // this unjam intake
            robot.light.purple();
            robot.intake.out();
            robot.feeder.intakeSlowFeed();

        } else {
            if (gamepad2.aWasReleased() || gamepad2.bWasReleased()) {
                robot.feeder.stopfeed();
                robot.intake.stop();
            }





            if (gamepad2.right_trigger > 0.5) {
//
                robot.shooter.updateShooterPID();
                robot.shooter.rpmSet(1150.0);
            }


            if (gamepad2.left_trigger > 0.5) {
//
                robot.shooter.updateShooterPIDFar();


                robot.shooter.rpmSet(1200.0);
            }

            if (gamepad2.y) {
                robot.stopshoot();
            } else if (manualTurn || gamepad1.a || !shotHeadingOkay || missingStartTarget) {
                if (robot.shooter.getLaunchState() != LaunchState.IDLE) {
                    robot.stopshoot();
                }
            } else {
                robot.shoot(shootPressed, 4, false, false);
            }





            if (gamepad2.xWasPressed()) {
                robot.setShooterVelocity(1145.0);
            }

        }

        if (gamepad1.y) {
            robot.upArm();
        }

        if (gamepad1.x) {
            robot.downArm();
        }





        if (gamepad1.b&& slowMode == false) {
            if (robot.drivetrain.getSlowMode()) {
                robot.drivetrain.setSlowMode(false);
            } else {
                robot.drivetrain.setSlowMode(true);
            }
        }

        if (gamepad1.b) {
            slowMode = true;
        } else {
            slowMode = false;
        }









        /** Bind Drivetrain **/
        LaunchState launchState = robot.shooter.getLaunchState();
        boolean feeding = launchState == LaunchState.LAUNCHING;
        double driveHeading = heldHeading;
        if (hiveLock && launchState != LaunchState.IDLE) {
            driveHeading = shotStartHeading;
            heldHeading = shotStartHeading;
            hiveHeading = Double.NaN;
        } else if (hiveLock && Double.isFinite(hiveHeading)) {
            driveHeading = hiveHeading;
        }
        double headingError = AngleUnit.normalizeDegrees(driveHeading - heading);
        double turnPower = 0;




        if (manualTurn) {
            turnPower = turnStick;
        } else if (Double.isFinite(headingError) && Double.isFinite(turnRate)) {
            double limit = Range.clip(maxTurnPower, 0, 1);
            turnPower = Range.clip(-headingP * headingError + headingD * turnRate, -limit, limit);
        }
        if (Double.isFinite(heading)) {
            robot.drivetrain.driveFieldRelative(driveValues.get(0), driveValues.get(1), turnPower, feeding);
        } else {
            robot.drivetrain.drive(-driveValues.get(0), -driveValues.get(1), turnPower);
        }

        // telemetry
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetryEssentials(telemetry, packet);
        telemetryIntake(telemetry, packet);
        //telemetryFeeder(telemetry, packet);
        telemetryShooter(telemetry, packet);
        telemetryShooting(telemetry, packet);
        if (!hiveLock) {
            telemetry.addData("Hive lock", "Off");
        } else if (launchState != LaunchState.IDLE) {
            telemetry.addData("Hive lock", "Holding shot heading");
        } else if (!hasTarget) {
            telemetry.addData("Hive lock", "Waiting for hive");
        } else {
            telemetry.addData("Hive lock", hiveAimPoint);
        }
        if (Double.isFinite(shotHeadingChange)) {
            telemetry.addData("Turn since Shoot", "%.1f deg", shotHeadingChange);
        }
        telementryRoadRunner(telemetry, packet);

        /**Start of Drive Train Information **/
        //telemetryDrivetrain(telemetry, packet);
        /**End of Drive Train Information**/

        telemetry.addData("Intake Velocity", robot.intake.getVel());

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    private void updateHiveAim(LLResult frame, boolean freshFrame, boolean canChooseAim, double heading) {
        if (!hiveLock || !freshFrame || gamepad1.a || !Double.isFinite(heading)) {
            hasHiveTarget = false;
            lastHiveFrame = Double.NaN;
        } else if (frame.getTimestamp() != lastHiveFrame) {
            lastHiveFrame = frame.getTimestamp();
            HiveTarget target = HiveTarget.find(frame.getFiducialResults(), robot.getIsBlue());
            hasHiveTarget = target != null;
            if (hasHiveTarget && canChooseAim) {
                hiveAimPoint = target.chooseAimPoint(hiveAimPoint, hiveCornerDifferenceInches);
                double offset = 0;
                if (hiveAimPoint == HiveTarget.AimPoint.LEFT) {
                    offset = hiveLeftAimOffsetDegrees;
                } else if (hiveAimPoint == HiveTarget.AimPoint.RIGHT) {
                    offset = hiveRightAimOffsetDegrees;
                }
                hiveHeading = heading - target.angleTo(hiveAimPoint) - offset;
            }
        }

        if (!hasHiveTarget && Double.isFinite(hiveHeading)) {
            if (Double.isFinite(heldHeading)) {
                heldHeading = heading;
            }
            hiveHeading = Double.NaN;
        }
    }

    private boolean isFreshTagFrame(LLResult frame) {
        if (frame == null || !frame.isValid() || !Double.isFinite(frame.getTimestamp())
                || frame.getTimestamp() <= 0 || frame.getStaleness() > 150) {
            return false;
        }

        long nowMs = System.nanoTime() / 1_000_000L;
        if (frame.getTimestamp() != lastTagFrame) {
            lastTagFrame = frame.getTimestamp();
            lastTagFrameMs = nowMs;
        }
        return nowMs - lastTagFrameMs <= 150;
    }


    @Override
    public void telemetryFeeder(Telemetry telemetry, TelemetryPacket packet) {
        double feederPower = robot.feeder.getPower();
        telemetry.addData("feeder Power", feederPower);
        packet.put("feeder Power", feederPower);

        double gatePos = robot.feeder.getGatePos();
        telemetry.addData("Gate Pos", gatePos);
        packet.put("Gate Pos", gatePos);
    }

    public void telemetryShooting(Telemetry telemetry, TelemetryPacket packet){

        LLResult frame = robot.vision.getLatestResult();
        double distanceToTag = -1;
        if (frame != null && frame.isValid() && frame.getFiducialResults() != null
                && !frame.getFiducialResults().isEmpty()) {
            distanceToTag = robot.getDistanceFromTag();
        }
        telemetry.addData("Distance to Tag", distanceToTag);
        packet.put("Distance to Tag", distanceToTag);
        telemetry.addData("Distance to Tag PODS", robot.getDistanceFromTagPODS());
        packet.put("Distance to Tag PODS", robot.getDistanceFromTagPODS());

        if (frame != null) {
            telemetry.addData("TagX", robot.getCameraTagX());
            packet.put("TagX", robot.getCameraTagX());
            telemetry.addData("TagY", robot.getCameraTagY());
            packet.put("TagY", robot.getCameraTagY());
        }
    }
}
