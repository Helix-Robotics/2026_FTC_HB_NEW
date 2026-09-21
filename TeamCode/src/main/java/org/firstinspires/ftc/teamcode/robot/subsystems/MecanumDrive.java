package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.Localizer;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

class PIDLoop{
    private double previousError;
    private double previousTime;
    private double previousOutput;

    private double errorR;

    public double calculateAxisPID(double error, double pGain, double dGain, double accel, double currentTime){
        double p = error * pGain;
        double cycleTime = currentTime - previousTime;
        double d = dGain * (previousError - error) / (cycleTime);
        double output = p + d;
        double dV = cycleTime * accel;

        double max = Math.abs(output);
        if(max > 1.0){
            output /= max;
        }

        if((output - previousOutput) > dV){
            output = previousOutput + dV;
        } else if ((output - previousOutput) < -dV){
            output = previousOutput - dV;
        }

        previousOutput = output;
        previousError  = error;
        previousTime   = currentTime;

        errorR = error;

        return output;
    }

}

@Config
public final class MecanumDrive {

    private static MecanumDrive driveTrain = null;

    /** Mecanum Drive Train **/
    private final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);
    private final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    private final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    private final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public final VoltageSensor voltageSensor;

    /**Localiser Related Stuff**/
    public final LazyImu lazyImu;
    // This declares the IMU needed to get the current direction the robot is facing
    private IMU imu;

    public final Localizer localizer;

    private final HelixLocalisation helixLocalizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    /**Debugging**/
    private double gyroRadians=0.0;
    private double gyroDegrees=0.0;

    /**Moce to Position**/
    private final PIDLoop xPID = new PIDLoop();
    private final PIDLoop yPID = new PIDLoop();
    private final PIDLoop hPID = new PIDLoop();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();
    private static double xyTolerance = 12;
    private static double yawTolerance = 0.0349066;
    private static double pGain = 0.008;
    private static double dGain = 0.00001;
    private static double accel = 10.0;
    private static double yawPGain = 5.0;
    private static double yawDGain = 0.0;
    private static double yawAccel = 20.0;

    private boolean slowMode = false;

    /*public static synchronized MecanumDrive getInstance(HardwareMap hardwareMap, Pose2d pose)
    {
        if (driveTrain == null)
            driveTrain = new MecanumDrive(hardwareMap, pose);

        driveTrain.setupMecanumDriveMotors();
        driveTrain.setupOtherConfigs(hardwareMap, pose);

        return driveTrain;
    }*/

    private enum Direction {
        x,
        y,
        h
    }

    private enum InBounds {
        NOT_IN_BOUNDS,
        IN_X_Y,
        IN_HEADING,
        IN_BOUNDS
    }

    /** VO ROBOT FINE TUNE**/
    /*
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        // If fixed the parameters
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        //public double inPerTick = 365.76/(2.54*1022) ; //365.75cm in 1022 ticks // Previous data: 136cm in 26156 ticks
        public double inPerTick = 1.0 / (19.89436789*25.4); //This basing on stats from gobilda setup //0.00502; //314.0/62604;
        //5/Nov lateral in per tick: 0.0014641518323635638, y-intercept: 0
        //5/Nov lateral in per tick: 0.0014965634076154089, y-intercept: 0
        //5/Nov lateral in per tick: 0.001489640197258577, y-intercept: 0
        public double lateralInPerTick = inPerTick; //0.001489640197258577; //inPerTick;

        //29/Oct track width: 43.70955933004513
        //30/Oct Track Width: 6387.485835048639
        //30/Oct track width: 6491.668772010902, y-intercept: 0
        //30/Oct track width: 6468.872448510864, y-intercept: 0
        //5/nov track width: 6393.7319061306425, y-intercept: 0
        //5/nov track width: 6361.589193451881, y-intercept: 0
        //5/nov track width: 6321.266741519434, y-intercept: 0
        public double trackWidthTicks = 6474.255791297305; //6474.255791297305; //6361.589193451881; //6468.872448510864;//6387.485835048639; //43.70955933004513;//0.8282637075365602;

        // feedforward parameters (in tick units)
        //kV: 0.008242484006246869, kS: 3.303279652453645 //tested on 29/Oct
        //kV: 0.015606884874459334, kS: 2.0553112060957197 //after excluding some points on 29/Oct
        //kV: 0.00026448360881591346, kS: 0.8901897675011892
        //kV: 0.000261782755880511, kS: 0.9232145996666627
        //30/Oct kV: 0.0002716050483263959, kS: 0.8527857400705834
        //30/Oct kV: 0.00026466599632218044, kS: 0.9166883563024251
        //30/OCT kV: 0.000268320986883893, kS: 0.8532716652383519
        //30/OCT kV: 0.0002682960378470928, kS: 0.83562042200563
        //30/OCT kV: 0.0002707327186137094, kS: 0.8047844545549783
        //5/Nov kV: 0.00027436562290601165, kS: 0.7290654823298408
        //5/Nov kV: 0.0002749101904376235, kS: 0.7270949505556845
        //5/Nov kV: 0.0002751037940384638, kS: 0.7186586632460394
        //5/Nov kV: 0.00026853212785488226, kS: 0.791668946584946
        public double kS = 0.800; //0.791668946584946; //0.7270949505556845; //0.8527857400705834;//2.0553112060957197; //2.202610700832471;//Previous Data:1.7468070152674622, 0.9027397917104008;
        public double kV = 0.000279; //0.00026853212785488226; //0.0002749101904376235; //0.0002716050483263959; //0.015606884874459334; //0.012272637616122635; //Previous Data: 0.010054732768618325, 0.00038880324921085444;
        //30/Oct KA = 0.00004
        //5/Nov KA = 0.000025
        public double kA = 0.000041; //0.000058;//0.000025; //0.000034; //0.00004; //0.001;

        // path profile parameters (in inches)
        public double maxWheelVel = 35; //40; //50;
        public double minProfileAccel = -14; //-24; //-30;
        public double maxProfileAccel = 20; //40; //50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 3.3; //2.9//9;
        public double lateralGain = 2.5; //7;
        public double headingGain = 1.5; //2; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }*/

    /** V1 Robot Config**/
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        // If fixed the parameters
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT; // Right before
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP; // was backwards before

        // drive model parameters
        public double inPerTick = 1.0 / (19.89436789*25.4); //This basing on stats from gobilda setup //0.00502; //314.0/62604;
        public double lateralInPerTick = inPerTick; //use gobilda config

        //11/Nov track width: 5975.4480602345975, y-intercept: 0
        public double trackWidthTicks = 5975.4480602345975;

        // feedforward parameters (in tick units)
        // 11/Nov -1 kV: 0.00019309119506222592, kS: 3.0857719939393746
        // 11/Nov -2 kV: 0.00022649552710333996, kS: 2.3840538328293186
        // 11/Nov -3 kV: 0.00027752052012891277, kS: 1.0527922657265067




        public double kS = 0.6387054295373398; //0.9182138486327918;  // 1.05240 before april 26
        public double kV = 0.00025829322013896087; //0.00026592358669776654;  // Before April 26 value 0.00023794
        public double kA = 0.000015;  //original value 0.000024

        // path profile parameters (in inches)
        public double maxWheelVel = 38; //50
        public double minProfileAccel = -20; //-40
        public double maxProfileAccel = 32; //50

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI ; // shared with path
        public double maxAngAccel = Math.PI; // *1.5

        // path controller gains
        public double axialGain = 6;
        public double lateralGain = 11.5;
        public double headingGain = 7.5;

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    /**We wont use Drive Localiser**/
    /**Instead we will pinpoint localiser**/
    public class DriveLocalizer implements Localizer {
        //Rename the encoder so that it will
        private final Encoder leftFrontEncoder, leftBackEncoder, rightBackEncoder, rightFrontEncoder;
        //move this to parent class
        //private final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFrontEncoder = new OverflowEncoder(new RawEncoder(leftFront));
            leftBackEncoder = new OverflowEncoder(new RawEncoder(leftBack));
            rightBackEncoder = new OverflowEncoder(new RawEncoder(rightBack));
            rightFrontEncoder = new OverflowEncoder(new RawEncoder(rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFrontEncoder.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBackEncoder.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBackEncoder.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFrontEncoder.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            return twist.velocity().value();
        }

        public List getEncoders(){
            return Arrays.asList(leftFrontEncoder, leftBackEncoder, rightFrontEncoder, rightBackEncoder);
        }
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                setDrivePowers(0,0,0,0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);

            if(txWorldTarget.value().position!=null&&txWorldTarget.value().heading!=null){
                //write(fbwehfiwfeuibf)
                targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            }

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            setDrivePowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                setDrivePowers(0,0,0,0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            setDrivePowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    private void setupMecanumDriveMotors(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);    //have tried forward for the new config
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
    }

    /*private void setupOtherConfigs(HardwareMap hardwareMap, Pose2d initialPos){
        helixLocalizer = new HelixLocalisation(hardwareMap, PARAMS.inPerTick, initialPos);
        localizer = helixLocalizer.getLocalizer();
    }*/

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        /*leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");*/
        /**We switch front and back, it is kind of weird now**/
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        setupMecanumDriveMotors();

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        helixLocalizer = new HelixLocalisation(hardwareMap, PARAMS.inPerTick, pose);
        localizer = helixLocalizer.getLocalizer();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        this.setDrivePowers(wheelVels.leftFront.get(0) / maxPowerMag, wheelVels.leftBack.get(0) / maxPowerMag,
                wheelVels.rightBack.get(0) / maxPowerMag, wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setSlowMode(boolean slowMode){
        this.slowMode = slowMode;
    }

    public boolean getSlowMode(){
        return this.slowMode;
    }

    public void setDrivePowers(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower){
        double factors = 1;
        if (this.slowMode){
            factors = 0.3;
        }
        leftFront.setPower(leftFrontPower*factors);
        leftBack.setPower(leftBackPower*factors);
        rightBack.setPower(rightBackPower*factors);
        rightFront.setPower(rightFrontPower*factors);
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        localizer.setPose(beginPose);
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate) {
        //make sure that we will get imu
        imu = lazyImu.get();

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(-forward, -right);
        double r = Math.hypot(right, forward);


        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftPower = maxSpeed * (frontLeftPower / maxPower);
        frontRightPower = maxSpeed * (frontRightPower / maxPower);
        backLeftPower = maxSpeed * (backLeftPower / maxPower);
        backRightPower = maxSpeed * (backRightPower / maxPower);

        //instead of set direction, now we set manually
        setDrivePowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }

    private double calculatePID(Pose2D currentPosition, Pose2D targetPosition, Direction direction){
        if(direction == Direction.x){
            double xError = targetPosition.getX(MM) - currentPosition.getX(MM);
            return xPID.calculateAxisPID(xError, pGain, dGain, accel,PIDTimer.seconds());
        }
        if(direction == Direction.y){
            double yError = targetPosition.getY(MM) - currentPosition.getY(MM);
            return yPID.calculateAxisPID(yError, pGain, dGain, accel, PIDTimer.seconds());
        }
        if(direction == Direction.h){
            double hError = targetPosition.getHeading(AngleUnit.RADIANS) - currentPosition.getHeading(AngleUnit.RADIANS);
            return hPID.calculateAxisPID(hError, yawPGain, yawDGain, yawAccel, PIDTimer.seconds());
        }
        return 0;
    }

    private InBounds inBounds (Pose2D currPose, Pose2D trgtPose){
        boolean xInBounds = currPose.getX(MM) > (trgtPose.getX(MM) - xyTolerance) && currPose.getX(MM) < (trgtPose.getX(MM) + xyTolerance);
        boolean yInBounds = currPose.getY(MM) > (trgtPose.getY(MM) - xyTolerance) && currPose.getY(MM) < (trgtPose.getY(MM) + xyTolerance);
        boolean hInBounds = currPose.getHeading(RADIANS) > (trgtPose.getHeading(RADIANS) - yawTolerance) &&
                currPose.getHeading(RADIANS) < (trgtPose.getHeading(RADIANS) + yawTolerance);

        if (xInBounds && yInBounds && hInBounds){
            return InBounds.IN_BOUNDS;
        } else if (xInBounds && yInBounds){
            return InBounds.IN_X_Y;
        } else if (hInBounds){
            return InBounds.IN_HEADING;
        } else
            return InBounds.NOT_IN_BOUNDS;
    }

    public boolean driveTo(Pose2D currentPosition, Pose2D targetPosition, double power, double holdTime) {
        boolean atTarget;

        double xPWR = calculatePID(currentPosition, targetPosition, Direction.x);
        double yPWR = calculatePID(currentPosition, targetPosition, Direction.y);
        double hOutput = calculatePID(currentPosition, targetPosition, Direction.h);

        double heading = currentPosition.getHeading(AngleUnit.RADIANS);
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);

        double xOutput = (xPWR * cosine) + (yPWR * sine);
        double yOutput = (xPWR * sine) - (yPWR * cosine);

        drive(-xOutput * power, -yOutput * power, hOutput * power);

        if(inBounds(currentPosition,targetPosition) == InBounds.IN_BOUNDS){
            atTarget = true;
        }
        else {
            holdTimer.reset();
            atTarget = false;
        }

        if(atTarget && holdTimer.time() > holdTime){
            return true;
        }
        return false;
    }

    public double getFRPower() {
        return rightFront.getPower();
    }

    public double getFLPower() {
        return leftFront.getPower();
    }

    public double getBRPower() {
        return rightBack.getPower();
    }

    public double getBLPower() {
        return leftBack.getPower();
    }

    public void resetImu() {
        lazyImu.get().resetYaw();
    }

    public void update(){
        imu = lazyImu.get();
        //we will do nothing for now
        gyroRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        gyroDegrees = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public double getGyroRadians() {
        imu = lazyImu.get();
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public HelixLocalisation getHelixLocalizer(){
        return helixLocalizer;
    }

    public Localizer getLocalizer(){
        return localizer;
    }

    //public Vision getVision(){
        //return helixLocalizer.getVision();   just for now
    //}

    public IMU getIMU(){
        return lazyImu.get();
    }

    public DcMotorEx getLeftFront(){
        return leftFront;
    }

    public DcMotorEx getLeftBack(){
        return leftBack;
    }

    public DcMotorEx getRightBack(){
        return rightBack;
    }

    public DcMotorEx getRightFront(){
        return rightFront;
    }

    public LazyImu getLazyImu(){
        return lazyImu;
    }
}