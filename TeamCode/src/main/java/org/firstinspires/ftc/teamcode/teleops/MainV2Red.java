package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        stateMachine = StateMachine.WAITING_FOR_START;
    }

    @Override
    public void loop() {
        // keep subsystems updated
        robot.update();
        robot.intake.update();
        robot.feeder.update();

        /** Driver Operations **/
        // drivetrain
        bindCommonDriveTrain();
        List<Double> driveValues = bindDriveTrain();

        //binding
        /**Above is only for testing**/
        robot.fieldRelativeDrive(driveValues.get(1), driveValues.get(0), driveValues.get(2));

        ElapsedTime timer = new ElapsedTime();

        /** Bind Drivetrain **/
        //binding Drive train
        if (gamepad1.aWasPressed()) {
            bindCommonDriveTrain();
        }

        /** Intake Related **/
        if (gamepad2.rightBumperWasPressed())
        {
            if (!outtaking)
            {
                intaking = false;
                outtaking = true;
                robot.intake.in();
                robot.feeder.slowfeed();
            }
            else
            {
                outtaking = false;
                robot.intake.stop();
                robot.feeder.stopfeed();
            }
        }
        if (gamepad2.leftBumperWasPressed()) {
            if (!intaking) {
                intaking = true;
                outtaking = false;
                robot.intake.out();
                robot.feeder.reverseFeed();
            } else {
                intaking = false;
                robot.intake.stop();
                robot.feeder.stopfeed();
            }
        }

        /** Shooting Related **/
        if (gamepad2.right_trigger > 0.5) {
            robot.shoot(true, 5);
        }
        if (gamepad2.left_trigger > 0.5){
            //Comment out fo debug
            //We dont kjnow what is impact
            robot.stopshoot();
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