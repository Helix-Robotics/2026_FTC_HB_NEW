package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        robot = new CommandsV2(hardwareMap, new Pose2d(0, 0, 0)) ;
        robot.feederDirection(true);
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

        //binding
        /**Above is only for testing**/
        robot.fieldRelativeDrive(driveValues.get(1), driveValues.get(0), driveValues.get(2));

        /** Bind Drivetrain **/
        //binding Drive train
        if (gamepad1.aWasPressed()) {
            bindCommonDriveTrain();
        }

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


        if (gamepad2.aWasPressed() || gamepad2.bWasPressed()) {
            robot.stopshoot();
        }

        if (gamepad2.a) {  // this unjam feeder
            robot.light.pink();
            robot.intake.hold();
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

            robot.shoot(gamepad2.right_trigger > 0.75, 5);

            if (gamepad2.left_trigger > 0.75) {
                robot.stopshoot();
            }
        }









        // telemetry
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetryEssentials(telemetry, packet);
        telemetryIntake(telemetry, packet);
        telemetryFeeder(telemetry, packet);
        telemetryShooter(telemetry, packet);
        telementryRoadRunner(telemetry, packet);

        /**Start of Drive Train Information **/
        //telemetryDrivetrain(telemetry, packet);
        /**End of Drive Train Information**/

        telemetry.addData("Intake Velocity", robot.intake.getVel());
        packet.put("Intake Vel", robot.intake.getVel());

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
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
}
