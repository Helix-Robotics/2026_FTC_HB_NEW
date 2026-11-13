package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.ShooterV4;

@TeleOp(name = "Shooter Spin Up Tuner", group = "Tuning")
public class ShooterSpinUpTuner extends OpMode {
    public static double P = ShooterV4.V3_SHOOTER_P;
    public static double D = ShooterV4.V3_SHOOTER_D;
    public static double F = ShooterV4.V3_SHOOTER_F;
    public static double TARGET_TPS = 1150;
    public static double TEST_DURATION_MS = 5000;

    private DcMotorEx shooter;
    private DcMotorEx feeder;
    private DcMotorEx intake;
    private Servo gate;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean running = false;
    private boolean timing = false;
    private String selected = "P";
    private double runTarget;
    private double runDurationMs;
    private double firstHitMs = -1;
    private double highestSpeed = 0;
    private String status = "Ready";

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        stop();
        telemetry.addLine("A: Start   Y: Stop   X: Select P, D or F");
        telemetry.addData("K value", "KP = %.2f", P);
        telemetry.addLine("D-pad up/down: Change by 0.5");
        telemetry.update();
    }

    @Override
    public void loop() {
        feeder.setPower(0);
        intake.setPower(0);
        gate.setPosition(0);
        double velocity = shooter.getVelocity();

        if (gamepad1.yWasPressed()) {
            stop();
            status = "Stopped";
        }

        boolean modeChanged = false;
        if (gamepad1.xWasPressed()) {
            if (selected.equals("P")) {
                selected = "D";
            } else if (selected.equals("D")) {
                selected = "F";
            } else {
                selected = "P";
            }
            modeChanged = true;
        }

        boolean gainChanged = false;
        if (gamepad1.dpadUpWasPressed()) {
            adjustGain(0.5);
            gainChanged = true;
        }
        if (gamepad1.dpadDownWasPressed()) {
            adjustGain(-0.5);
            gainChanged = true;
        }
        if (gainChanged || modeChanged) {
            timing = false;
            firstHitMs = -1;
            highestSpeed = velocity;
            if (running) {
                applyPIDF();
                status = "Running";
            }
        }

        if (!running) {
            if (gamepad1.aWasPressed() && !gamepad1.y) {
                if (Math.abs(velocity) > 1) {
                    status = "Wait for shooter to stop, then press A";
                } else if (!validSettings()) {
                    status = "Check settings";
                } else {
                    runTarget = TARGET_TPS;
                    runDurationMs = TEST_DURATION_MS;
                    firstHitMs = -1;
                    highestSpeed = 0;
                    shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    applyPIDF();
                    timer.reset();
                    shooter.setVelocity(runTarget);
                    running = true;
                    timing = selected.equals("P");
                    status = "Running";
                    velocity = shooter.getVelocity();
                }
            }
        }

        if (running) {
            double nowMs = timer.milliseconds();
            if (!Double.isFinite(velocity)) {
                stop();
                status = "Speed reading error";
            } else {
                highestSpeed = Math.max(highestSpeed, velocity);
                if (timing) {
                    if (velocity >= runTarget) {
                        firstHitMs = nowMs;
                        timing = false;
                        status = "Target reached; still running";
                    } else if (nowMs >= runDurationMs) {
                        timing = false;
                        status = "Target not reached in time; still running";
                    }
                }
            }
        }

        telemetry.addData("Status", status);
        if (selected.equals("F")) {
            telemetry.addData("K value", "KF = %.2f", F);
            telemetry.addData("KF Speed", "%.1f ticks/s", TARGET_TPS);
        } else if (selected.equals("D")) {
            telemetry.addData("K value", "KD = %.2f", D);
            telemetry.addData("Target speed", "%.1f ticks/s", TARGET_TPS);
            telemetry.addData("Highest speed", "%.1f ticks/s", highestSpeed);
        } else {
            telemetry.addData("K value", "KP = %.2f", P);
            telemetry.addData("Target speed", "%.1f ticks/s", TARGET_TPS);
            if (firstHitMs >= 0) telemetry.addData("Spin up time", "%.1f ms", firstHitMs);
            else if (running && !timing) telemetry.addData("Spin up time", "Restart to measure");
            else telemetry.addData("Spin up time", "Not reached");
        }
        telemetry.addData("Current speed", "%.1f ticks/s", velocity);
        telemetry.addLine("A: Start   Y: Stop   X: Select");
        telemetry.addLine("Up/down: +/- 0.5");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target speed", TARGET_TPS);
        packet.put("Current speed", velocity);
        packet.put("Running", running);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void applyPIDF() {
        if (selected.equals("F")) {
            shooter.setVelocityPIDFCoefficients(0, 0, 0, F);
        } else if (selected.equals("P")) {
            shooter.setVelocityPIDFCoefficients(P, 0, 0, F);
        } else {
            shooter.setVelocityPIDFCoefficients(P, 0, D, F);
        }
    }

    private void adjustGain(double change) {
        if (selected.equals("P")) {
            P = Math.max(0, P + change);
        } else if (selected.equals("D")) {
            D = Math.max(0, D + change);
        } else {
            F = Math.max(0, F + change);
        }
    }

    private boolean validSettings() {
        return Double.isFinite(P) && P >= 0 && Double.isFinite(D) && D >= 0
                && Double.isFinite(F) && F >= 0 && Double.isFinite(TARGET_TPS) && TARGET_TPS > 0
                && Double.isFinite(TEST_DURATION_MS) && TEST_DURATION_MS > 0;
    }

    @Override
    public void stop() {
        running = false;
        timing = false;
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);
        feeder.setPower(0);
        intake.setPower(0);
        gate.setPosition(0);
    }

    /*
     * F: Press A, let speed settle, then adjust KF until current speed stays close
     * to target. F mode automatically disables P and D.
     *
     * P: Compare several runs per KP value, rather than selecting the single
     * fastest result. Choose a fast, repeatable setting, then verify it does not
     * cause excessive overshoot or poor recovery during shooting.
     *
     * D: Keep your chosen P and F fixed. Start D at zero and increase it only if
     * it reduces overshoot or oscillation. If it adds noise or slows recovery,
     * reduce it.
     *
     * Do not use D to compensate for an excessively high P. Reduce P first if
     * the shooter repeatedly swings above and below target.
     *
     * A starts from rest. All modes keep running until Y is pressed.
     * X selects P, D or F. D-pad up/down adjusts the selected value by 0.5 live.
     * P mode disables D. I stays zero in every mode.
     * After changing a gain or mode, press Y, wait for rest, then A for a new
     * spin up time. Adjusting during a run clears the old measurement.
     * Copy your final gains into ShooterV4; this tuner does not save them there.
     */
}
