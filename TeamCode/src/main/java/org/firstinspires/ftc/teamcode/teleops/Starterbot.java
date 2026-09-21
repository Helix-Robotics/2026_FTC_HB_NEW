/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot Chassis/Intake for the
 * 2026-2027 FIRST® Tech Challenge. It leverages a differential/Skid-Steer system for robot mobility,
 * one motor driving an intake roller, and two servos which pull elements out of corners.
 */

@TeleOp(name = "Starterbot code")
//@Disabled
public class Starterbot extends OpMode {

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private DcMotorEx elevator = null;
    private CRServo door = null;
    //private CRServo leftIntakeServo = null;
    //private CRServo rightIntakeServo = null;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private static final double TICKS_PER_REV = 1425.1;
    private static final double DEGREES_PER_REV = 360.0;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / DEGREES_PER_REV;

    private static final double MIN_ANGLE = 0;
    private static final double MAX_ANGLE = 100000; //140; //162;
    private double targetDegrees = 0;

    // Set up a variable for each drive wheel to save power level for telemetry.
    double leftPower;
    double rightPower;

    // Create a variable to set to the intake.
    double intakePower;
    double elevatorPos;
    double speedmod;
    boolean intaking = false;
    boolean outtaking = false;

    public static double P = 1;
    public static double I = 0;
    public static double D = 0;
    public static double F = 70;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        door = hardwareMap.get(CRServo.class, "door");
        //leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        //rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        elevator.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        //leftIntakeServo.setPower(0);
        //rightIntakeServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        //rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        packet.put("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        arcadeDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x);
        elevatorPos = elevator.getCurrentPosition();
        // intakePower = gamepad1.right_trigger * 0.9 - gamepad1.left_trigger * 0.9;
        speedmod = 1;
        if (gamepad1.rightBumperWasPressed())
        {
            if (!intaking)
            {
                intaking = true;
                outtaking = false;
                intakePower = 1;
            }
            else
            {
                intaking = false;
                intakePower = 0;
            }
        }
        if (gamepad1.leftBumperWasPressed())
        {
            if (!outtaking)
            {
                intaking = false;
                outtaking = true;
                intakePower = -1;
            }
            else
            {
                outtaking = false;
                intakePower = 0;
            }
        }
        if (gamepad1.left_trigger > 0) {
            elevator.setPower(0.6);
            //setElevatorPIDF();
            //setTargetPosition(1150);
        }
        if (gamepad1.right_trigger > 0) {
            elevator.setPower(-0.6);
            //setElevatorPIDF();
            //setTargetPosition(0);
        }
        if (gamepad1.a) {
            elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        if (!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.a) // change 1 and 0 after, they're just placeholder
        {
            elevator.setPower(0);
        }

        if (gamepad1.dpad_down) {
            door.setPower(1);
        }
        if (gamepad1.dpad_up) {
            door.setPower(-1);
        }
        if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
            door.setPower(0);
        }
        if (gamepad1.a)
        {
            speedmod = 0.3;
        }
        if (gamepad1.b)
        {
            speedmod = 1;
        }
        intake.setPower(intakePower);
        //leftIntakeServo.setPower(intakePower);
        //rightIntakeServo.setPower(intakePower);





        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Intaking:", intaking);
        telemetry.addData("Outtaking:", outtaking);
        packet.put("Intaking:", intaking);
        packet.put("Outtaking:", outtaking);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        packet.put("leftPower:", leftPower);
        packet.put("rightPower:", rightPower);
        // telemetry.addData("Intake", "left (%.2f, right (%.2f)",gamepad1.left_trigger, gamepad1.right_trigger);
        packet.put("intake", intakePower);
        telemetry.addData("elevator height", elevatorPos);
        packet.put("elevator height", elevatorPos);
        telemetry.addData("Speedmod:", speedmod);
        packet.put("speedmod:", speedmod);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward - rotate;
        rightPower = forward + rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower * speedmod);
        rightDrive.setPower(rightPower * speedmod);
    }

    public void setTargetPosition(double degrees) {


        double currentPosition = elevator.getCurrentPosition();
        if (degrees < MIN_ANGLE) degrees = MIN_ANGLE;
        if (degrees > MAX_ANGLE) degrees = MAX_ANGLE;

        targetDegrees = degrees;
        int ticks = (int) Math.round(degrees * TICKS_PER_DEGREE);


        if (ticks == currentPosition){
            elevator.setPower(0.05);
        }
        else if (ticks < currentPosition){
            elevator.setPower(-0.6);
        }else{
            elevator.setPower(0.6);
        }
        elevator.setTargetPosition(ticks);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setElevatorPIDF() {
        elevator.setVelocityPIDFCoefficients(P, I, D, F);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPositionPIDFCoefficients(P);
    }


    public void resetPosition(){
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setVelocity(-300);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}