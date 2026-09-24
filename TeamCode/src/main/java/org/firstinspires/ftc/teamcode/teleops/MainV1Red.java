package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.robot.subsystems.HelixLocalisation.localizer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.CommandsV1;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

import java.util.List;


@TeleOp(name = "V1 Teleop")
public class MainV1Red extends MainV0Red {

    Intake intake;
    boolean intaking = false;

    boolean outtaking = false;
    public double intakePower = 0;




  
    @Override
    public void init() {
        // tune inPerTick for ur drivetrain encoders
        robot = new CommandsV1(hardwareMap, new Pose2d(0, 0, 0));
        stateMachine = StateMachine.WAITING_FOR_START;
        robot.feederDirection(true);
    }

    @Override
    public void loop() {
        // keep subsystems updated
        robot.update();
        robot.intake.update();

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
        else {
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


    public  void telemetryEssentials(Telemetry telemetry, TelemetryPacket packet){

    }

    public void telemetryIntake(Telemetry telemetry, TelemetryPacket packet) {
        double intakePower = robot.intake.getPower();
        telemetry.addData("Intake Power", intakePower);
        packet.put("Intake Power", intakePower);
    }

    public void telemetryFeeder(Telemetry telemetry, TelemetryPacket packet) {
        double feederPower = robot.feeder.getPower();
        telemetry.addData("Feeder Power", feederPower);
        packet.put("Feeder Power", feederPower);
    }

    public void telemetryShooter(Telemetry telemetry, TelemetryPacket packet) {
        double shootvel = robot.shooter.getVelocity();
        telemetry.addData("Shooter Velocity", shootvel);
        packet.put("Shooter Velocity", shootvel);

        Shooter.LaunchState state = robot.shooter.getLaunchState();
        telemetry.addData("Shooter State", state);
        packet.put("Shooter State", state);
    }

    public void telementryRoadRunner(Telemetry telemetry, TelemetryPacket packet) {
        Pose2d pose = localizer.getPose();

        telemetry.addData("Robot X inch", pose.position.x);
        packet.put("Robot X inch", pose.position.x);

        telemetry.addData("Robot Y inches", pose.position.y);
        packet.put("Robot Y inches", pose.position.y);

        int ticksX = localizer.driver.getEncoderX();
        int ticksY = localizer.driver.getEncoderY();
        telemetry.addData("Robot X ticks", ticksX);
        packet.put("Robot X ticks", ticksX);
        telemetry.addData("Robot Y ticks", ticksY);
        packet.put("Robot Y ticks", ticksY);

        double headingDegrees = Math.toDegrees(pose.heading.toDouble());

        telemetry.addData("Heading", headingDegrees);
        packet.put("Heading", headingDegrees);
    }
}
