package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.MAX_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.MIN_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.util.utilMethods.wrap360;

@Config
public class DriveTrain implements Subsystem {

    // motors
    public DcMotorEx motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, duckSpinner;
    private DcMotorEx[] motors;

    // sensors
    private BNO055IMU imu;
    private DistanceSensor sensorChassisDistance;

    // kinematics
    private SimpleMatrix pose; // [x, y, yaw]
    private SimpleMatrix velocity; // [vx, vy, w]
    private SimpleMatrix angles; // [heading, roll, pitch]
    private SimpleMatrix offsetAngles; // [heading, roll, pitch]
    private SimpleMatrix previousWheelTicks; // [left, right, middle]

    // state
    private double targetFrontLeftVelocity, targetFrontRightVelocity, targetMiddleVelocity, targetSwivelAngle;
    private double targetLinearVelocity, targetAngularVelocity, targetTurnRadius;

    private double swivelAngle;
    private double chassisDistance, targetChassisDistance;

    private boolean middleReversed;

    // PID
    private PIDController turnPID, drivePID, distPID, swivelPID, chassisDistancePID;
    private double maintainSwivelAngleCorrection, maintainChassisDistanceCorrection;
    private boolean maintainChassisDistanceEnabled, maintainSwivelAngleEnabled;

    // smoothers
    private ExponentialSmoother frontLeftSmoother;
    private ExponentialSmoother frontRightSmoother;
    private ExponentialSmoother middleSmoother;

    // constants
    public static final String TELEMETRY_NAME = "Drive Train";

    public static PIDCoefficients DRIVE_PID_COEFFICIENTS = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients ROTATE_PID_COEFFICIENTS = new PIDCoefficients(0.005, 0, .13);
    public static PIDCoefficients SWIVEL_PID_COEFFICIENTS = new PIDCoefficients(0.03, 0, 0.08);
    public static PIDCoefficients DIST_PID_COEFFICIENTS = new PIDCoefficients(2.0, 0, 0.5);
    public static PIDCoefficients CHASSIS_DISTANCE_PID_COEFFICIENTS = new PIDCoefficients(8, 0,  5);
    public static double SWIVEL_PID_TOLERANCE = 10;

    public static double FRONT_LEFT_SMOOTHING_FACTOR = 0.1;
    public static double FRONT_RIGHT_SMOOTHING_FACTOR = 0.1;
    public static double MIDDLE_SMOOTHING_FACTOR = 0.1;

    public static double DISTANCE_SENSOR_TO_FRONT_AXLE = 0.07;
    public static double DISTANCE_TARGET_TO_BACK_WHEEL = 0.18;

    public static double SWERVE_TICKS_PER_REVOLUTION = 1740;
    public static double TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev();
    public static double TICKS_PER_METER = TICKS_PER_REVOLUTION / (2 * Math.PI * Constants.WHEEL_RADIUS); // TODO: use TPM_CALIBRATION game state to calibrate TPM

    // threshold to buffer from max chassis length when attempting to fully extend chassis
    // (to not put excessive strain on linear slide)
    public static double CHASSIS_LENGTH_THRESHOLD = 0.1;

    public DriveTrain(HardwareMap hardwareMap) {
        String[] MOTOR_NAMES = {"motorFrontLeft", "motorFrontRight", "motorMiddle", "motorMiddleSwivel", "duckSpinner"};
        ZeroPowerBehavior[] ZERO_POWER_BEHAVIORS = new ZeroPowerBehavior[]{ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.BRAKE, ZeroPowerBehavior.BRAKE};
        boolean[] REVERSED = {true, false, true, false, false};

        // Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorMiddle= hardwareMap.get(DcMotorEx.class, "motorMiddle");
        motorMiddleSwivel = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
        duckSpinner = hardwareMap.get(DcMotorEx.class,"duckSpinner");
        motors = new DcMotorEx[] {motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, duckSpinner};

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(ZERO_POWER_BEHAVIORS[i]);
            if (REVERSED[i])
                motors[i].setDirection(Direction.REVERSE);
        }

        motorMiddleSwivel.setMode(RunMode.RUN_WITHOUT_ENCODER);

        // Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        sensorChassisDistance = hardwareMap.get(DistanceSensor.class, "distLength");

        // Kinematics
        pose = new SimpleMatrix(3, 1);
        velocity = new SimpleMatrix(3, 1);
        angles = new SimpleMatrix(3, 1);
        offsetAngles = new SimpleMatrix(3, 1);
        previousWheelTicks = new SimpleMatrix(3, 2);

        // PID
        turnPID = new PIDController(ROTATE_PID_COEFFICIENTS);
        drivePID = new PIDController(DRIVE_PID_COEFFICIENTS);
        swivelPID = new PIDController(SWIVEL_PID_COEFFICIENTS);
        distPID = new PIDController(DIST_PID_COEFFICIENTS);
        chassisDistancePID = new PIDController(CHASSIS_DISTANCE_PID_COEFFICIENTS);

        // Smoother
        frontLeftSmoother = new ExponentialSmoother(FRONT_LEFT_SMOOTHING_FACTOR);
        frontRightSmoother = new ExponentialSmoother(FRONT_RIGHT_SMOOTHING_FACTOR);
        middleSmoother = new ExponentialSmoother(MIDDLE_SMOOTHING_FACTOR);

        // Miscellaneous
        previousWheelTicks = getWheelTicks();
        maintainSwivelAngleEnabled = true;
    }

    private void initializeIMU() {
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "baseIMU";

        imu.initialize(parametersIMU);

        // storing first absolute orientation values as offsets
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        offsetAngles = new SimpleMatrix(new double[][] {
                { (360 - imuAngles.firstAngle) % 360 },
                { imuAngles.secondAngle % 360 },
                { imuAngles.thirdAngle % 360 }
        });
    }

    private double getMaintainSwivelAngleCorrection() {
        //initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(SWIVEL_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetSwivelAngle);
        swivelPID.setTolerance(SWIVEL_PID_TOLERANCE);
        swivelPID.enable();

        //initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, 360);
        swivelPID.setContinuous(true);
        swivelPID.setInput(swivelAngle);

        //calculates the angular correction to apply
        return swivelPID.performPID();
    }

    private double getMaintainChassisDistanceCorrection() {
        // initialization of the PID calculator's output range, target value and multipliers
        chassisDistancePID.setOutputRange(-5.0, 5.0);
        chassisDistancePID.setPID(CHASSIS_DISTANCE_PID_COEFFICIENTS);
        chassisDistancePID.setSetpoint(targetChassisDistance);
        chassisDistancePID.enable();

        // initialization of the PID calculator's input range and current value
        chassisDistancePID.setInputRange(Constants.MIN_CHASSIS_LENGTH, Constants.MAX_CHASSIS_LENGTH);
        chassisDistancePID.setInput(chassisDistance);

        // calculating correction
        return chassisDistancePID.performPID();
    }

    /**
     * updates the robot's pose ((x,y) position and heading) using the encoder ticks travelled by each wheel motor
     */
    private void updatePose() {
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angles = new SimpleMatrix(new double[][] {
                {UtilMethods.wrapAngle(360 - imuAngles.firstAngle, offsetAngles.get(0))},
                {UtilMethods.wrapAngle(imuAngles.thirdAngle, offsetAngles.get(1))},
                {UtilMethods.wrapAngle(imuAngles.secondAngle, offsetAngles.get(2))}
        });

        // calculating wheel displacements
//        SimpleMatrix wheelTicks = getWheelTicks().rows(0, 2);
//        SimpleMatrix wheelDisplacementMeters = wheelTicks.minus(previousWheelTicks.rows(0, 2)).divide(TICKS_PER_METER);

//        // rotating swivel wheel by swivel angle
//        double swivelAngle = getSwivelAngle();
//        SimpleMatrix swivelWheel = MathUtil.rotateVector(
//                new SimpleMatrix(
//                        new double[][] {{ wheelDisplacementMeters.get(2, 0), 0 }}
//                ),
//                Math.toRadians(swivelAngle)
//        );
//        wheelDisplacementMeters.setRow(2, 0, swivelWheel.get(0), swivelWheel.get(1));

        // calculating average average wheel displacement
//        SimpleMatrix ones = new SimpleMatrix(new double[][] {{1, 1}});
//        SimpleMatrix averageDisplacementMeters = ones.mult(wheelDisplacementMeters).divide(2);

        // rotating displacement by heading
        double heading = angles.get(0);
//        averageDisplacementMeters = UtilMethods.rotateVector(averageDisplacementMeters, Math.toRadians(heading));

        // updating pose [x, y, heading]
//        pose = pose.plus(new SimpleMatrix(new double[][] {{
//            averageDisplacementMeters.get(0, 0),
//            averageDisplacementMeters.get(1, 0),
//            0
//        }}).transpose());
        pose.set(2, 0, angles.get(0));

//        previousWheelTicks = wheelTicks.copy();
    }

    @Override
    public void update() {
        // state
        chassisDistance = sensorChassisDistance.getDistance(DistanceUnit.MM) / 1000 + DISTANCE_SENSOR_TO_FRONT_AXLE + DISTANCE_TARGET_TO_BACK_WHEEL;
        swivelAngle = (motorMiddleSwivel.getCurrentPosition() / SWERVE_TICKS_PER_REVOLUTION * 360) % 360;

        // PID corrections
        if(maintainSwivelAngleEnabled)
            maintainSwivelAngleCorrection = getMaintainSwivelAngleCorrection();
        motorMiddleSwivel.setPower(maintainSwivelAngleCorrection);

//        updateTargetChassisDistance();
        if(maintainChassisDistanceEnabled) {
            maintainChassisDistanceCorrection = getMaintainChassisDistanceCorrection();
            targetFrontLeftVelocity += maintainChassisDistanceCorrection;
            targetFrontRightVelocity += maintainChassisDistanceCorrection;
        }

        if(swivelAngle > 180 && swivelAngle < 360) {
//            targetMiddleVelocity = -targetMiddleVelocity;
        }


        // Motor controls
//        if(swivelPID.onTarget()) {
            motorFrontLeft.setVelocity(targetFrontLeftVelocity * TICKS_PER_METER);
            motorFrontRight.setVelocity(targetFrontRightVelocity * TICKS_PER_METER);
            motorMiddle.setVelocity(targetMiddleVelocity * TICKS_PER_METER);
//        } else {
//            motorFrontLeft.setVelocity(0);
//            motorFrontRight.setVelocity(0);
//            motorMiddle.setVelocity(0);
//        }

//        updatePose();
    }

    private void handleSmoothing() {
        targetFrontLeftVelocity = frontLeftSmoother.update(targetFrontLeftVelocity);
        targetFrontRightVelocity = frontRightSmoother.update(targetFrontLeftVelocity);
        targetMiddleVelocity = middleSmoother.update(targetMiddleVelocity);
    }

    /**
     * Drives the robot with the specified linear and angular velocities
     * @param linearVelocity the velocity, in m/s, to drive the robot
     * @param angularVelocity the angular velocity, in rad/s, to drive the robot
     */
    public void drive(double linearVelocity, double angularVelocity, boolean smoothingEnabled) {
        targetLinearVelocity = linearVelocity;
        targetAngularVelocity = angularVelocity;

        targetTurnRadius = (angularVelocity == 0 ? 0 : linearVelocity / angularVelocity);

        targetFrontLeftVelocity = linearVelocity + angularVelocity * (targetTurnRadius - Constants.TRACK_WIDTH / 2);
        targetFrontRightVelocity = linearVelocity + angularVelocity * (targetTurnRadius + Constants.TRACK_WIDTH / 2);
        targetMiddleVelocity = linearVelocity + angularVelocity * Math.hypot(targetTurnRadius, chassisDistance);


        targetSwivelAngle = (angularVelocity == 0 || (angularVelocity == 0 && linearVelocity == 0))
                ? 90
                    : linearVelocity == 0
                    ? 0
                : 90 - Math.atan2(chassisDistance, targetTurnRadius);

        // angularVelocity and linear
        if(smoothingEnabled)
            handleSmoothing();
    }

    public void driveDesmos(double linearVelocity, double angularVelocity, double dt, boolean smoothingEnabled) {
        targetLinearVelocity = linearVelocity;
        targetAngularVelocity = angularVelocity;

        targetTurnRadius = angularVelocity == 0 ? 0 : linearVelocity / angularVelocity;

        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 , 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2, 0 }});
        SimpleMatrix middleWheel = new SimpleMatrix(new double[][] {{ 0, -getChassisDistance() }});

        SimpleMatrix translation = new SimpleMatrix(new double[][] {{ 0, linearVelocity * dt }});

        SimpleMatrix leftWheelPrime = translation.plus(UtilMethods.rotateVector(leftWheel, angularVelocity * dt).transpose());
        SimpleMatrix rightWheelPrime = translation.plus(UtilMethods.rotateVector(rightWheel, angularVelocity * dt).transpose());
        SimpleMatrix middleWheelPrime = translation.plus(UtilMethods.rotateVector(middleWheel, -angularVelocity * dt).transpose());

        targetFrontLeftVelocity = Math.signum(leftWheelPrime.get(1)) * leftWheelPrime.minus(leftWheel).normF() / dt;
        targetFrontRightVelocity = Math.signum(rightWheelPrime.get(1)) * rightWheelPrime.minus(rightWheel).normF() / dt;
        targetMiddleVelocity = Math.signum(middleWheelPrime.get(1) - middleWheel.get(1)) * middleWheelPrime.minus(middleWheel).normF() / dt;

        targetSwivelAngle = UtilMethods.wrapAngle(Math.toDegrees(Math.atan2(middleWheelPrime.get(1) - middleWheel.get(1), middleWheelPrime.get(0) - middleWheel.get(0))));

        if(UtilMethods.wrapAngle(targetSwivelAngle - swivelAngle) > 90) {
            middleReversed = true;
            targetMiddleVelocity = -1 * targetMiddleVelocity;
            targetSwivelAngle = UtilMethods.wrapAngle(180 - targetSwivelAngle);
        } else
            middleReversed = false;

        if(smoothingEnabled)
            handleSmoothing();
    }

    public void movePID(double maxPwrFwd, boolean forward, double dist, double currentAngle, double targetAngle) {
        // setup turnPID
        turnPID.setOutputRange(-.5, .5);
        turnPID.setIntegralCutIn(1);
        turnPID.setSetpoint(targetAngle);
        turnPID.setInputRange(0, 360);
        turnPID.setContinuous();
        turnPID.setInput(currentAngle);
        turnPID.enable();

        // setup distPID
        distPID.setOutputRange(-maxPwrFwd, maxPwrFwd);
        distPID.setIntegralCutIn(1);
        distPID.setSetpoint(dist); //trying to get to a zero distance
        distPID.setInput(0);
        distPID.enable();

        // calculate the angular correction to apply
        double turnCorrection = turnPID.performPID();
        // calculate chassis power
        double basePwr = distPID.performPID();
        if (!forward) basePwr *=-1;

        // performs the drive with the correction applied

        drive(basePwr, turnCorrection, false);
    }

    public boolean driveAbsoluteDistance(double pwr, double targetAngle, boolean forward, double targetMeters, double closeEnoughDist) {
        targetAngle= wrap360(targetAngle);  //this was probably already done but repeated as a safety

        if (Math.abs(targetMeters) > Math.abs(closeEnoughDist)) {
            movePID(pwr, forward, targetMeters,getHeading(),targetAngle);
            return false;
        } // destination achieved
        else {
            stop(); //todo: maybe this should be optional when you are stringing moves together
            return true;
        }
    }

    private long turnTimer = 0;
    private boolean turnTimerInit = false;
    private double minTurnError = 2.0;
    public boolean rotateIMU(double targetAngle, double maxTime) {
        if (!turnTimerInit) { // intiate the timer that the robot will use to cut of the sequence if it takes
            // too long; only happens on the first cycle
            turnTimer = System.nanoTime() + (long) (maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        movePID(1,true,0, getHeading(), targetAngle);
        // threshold of the target
        if(Math.abs(getHeading() - targetAngle) < minTurnError) {
            turnTimerInit = false;
            stop();
            return true;
        }

        if (turnTimer < System.nanoTime()) { // check to see if the robot takes too long to turn within a threshold of
            // the target (e.g. it gets stuck)
            turnTimerInit = false;
            stop();
            return true;
        }
        return false;
    }

    public boolean handleDuckSpinner(double power){
        duckSpinner.setPower(power);
        return true;
    }

    boolean duckSpinnerIsOn = false;
    public boolean handleDuckSpinnerToggle(int mod) {
        if(duckSpinnerIsOn) {
            handleDuckSpinner(0);
            duckSpinnerIsOn = false;
        }
        else{
            handleDuckSpinner(mod * .5);
            duckSpinnerIsOn = true;
        }

        return true;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        if(debug) {
            telemetryMap.put("fl position", motorFrontLeft.getCurrentPosition());
            telemetryMap.put("fr position", motorFrontRight.getCurrentPosition());
            telemetryMap.put("middle position", motorMiddle.getCurrentPosition());
            telemetryMap.put("swivel position", motorMiddleSwivel.getCurrentPosition());

            telemetryMap.put("fl velocity", ticksToMeters(motorFrontLeft.getVelocity()));
            telemetryMap.put("fr velocity", ticksToMeters(motorFrontRight.getVelocity()));
            telemetryMap.put("middle velocity", ticksToMeters(motorMiddle.getVelocity()));

            telemetryMap.put("target turn radius", targetTurnRadius);
            telemetryMap.put("target linear velocity", targetLinearVelocity);
            telemetryMap.put("target angular velocity", targetAngularVelocity);

            telemetryMap.put("fl target velocity", targetFrontLeftVelocity);
            telemetryMap.put("fr target velocity", targetFrontRightVelocity);
            telemetryMap.put("middle target velocity", targetMiddleVelocity);
            telemetryMap.put("swivel target power", maintainSwivelAngleCorrection);
            telemetryMap.put("duck power", duckSpinner.getPower());
            telemetryMap.put("duck position", duckSpinner.getCurrentPosition());

            telemetryMap.put("fl amps", motorFrontLeft.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("fr amps", motorFrontRight.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("middle amps", motorMiddle.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("swivel amps", motorMiddleSwivel.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("duck amps", duckSpinner.getCurrent(CurrentUnit.AMPS));
            telemetryMap.put("atan2 ", Math.atan2(chassisDistance, targetTurnRadius));
            telemetryMap.put("90 - atan2 to deg ", 90 - Math.toDegrees(Math.atan2(chassisDistance, targetTurnRadius)));
            telemetryMap.put("turn radius", targetTurnRadius);

            telemetryMap.put("swivel angle", swivelAngle);
            telemetryMap.put("target swivel angle", targetSwivelAngle);
            telemetryMap.put("swivel PID on target", swivelPID.onTarget());

            telemetryMap.put("chassis distance", chassisDistance);
            telemetryMap.put("target chassis distance", targetChassisDistance);
            telemetryMap.put("maintain chassis distance enabled", maintainChassisDistanceEnabled);
            telemetryMap.put("maintain chassis distance correction", maintainChassisDistanceCorrection);

            telemetryMap.put("pose (x)", pose.get(0));
            telemetryMap.put("pose (y)", pose.get(1));
            telemetryMap.put("pose (heading)", Math.toDegrees(pose.get(2)));

            telemetryMap.put("middle wheel reversed", middleReversed);
        }

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public double getSwivelAngle() {
        return swivelAngle;
    }

    public double getChassisDistance() {
        return chassisDistance;
    }

    public SimpleMatrix getPose() {
        return pose;
    }

    public double getHeading(){
        return Math.toDegrees(pose.get(2));
    }

    public SimpleMatrix getWheelTicks() {
        return new SimpleMatrix(new double[][] {
                { motorFrontLeft.getCurrentPosition(), 0 },
                { motorFrontRight.getCurrentPosition(), 0 },
                { motorMiddle.getCurrentPosition(), 0 }
        });
    }

    public double getTurnRadius() {
        return targetTurnRadius;
    }

    public void setMaintainChassisDistanceEnabled(boolean maintainChassisDistanceEnabled) {
        this.maintainChassisDistanceEnabled = maintainChassisDistanceEnabled;
    }

    public void setTargetChassisDistance(double targetChassisDistance) {
        this.targetChassisDistance = Range.clip(targetChassisDistance, MIN_CHASSIS_LENGTH + CHASSIS_LENGTH_THRESHOLD, MAX_CHASSIS_LENGTH - CHASSIS_LENGTH_THRESHOLD);
    }

    public static int metersToTicks(double meters) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        double revolutions = meters / circumference;
        return (int) (revolutions * TICKS_PER_REVOLUTION);
    }

    public static double ticksToMeters(double ticks) {
        double revolutions = ticks / TICKS_PER_REVOLUTION;
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return revolutions * circumference;
    }

    public void setFrontLeftTargetVelocity(double velocity) {
        targetFrontLeftVelocity = velocity;
    }
    public void setFrontRightTargetVelocity(double velocity) {
        targetFrontRightVelocity = velocity;
    }
    public void setMiddleTargetVelocity(double velocity) {
        targetMiddleVelocity = velocity;
    }
    public void setSwivelTargetAngle(double targetAngle) {
        targetSwivelAngle = targetAngle;
    }

    public double getSwivelTargetAngle() {
        return targetSwivelAngle;
    }

    public void setMaintainSwivelAngleEnabled(boolean maintainSwivelAngleEnabled) {
        this.maintainSwivelAngleEnabled = maintainSwivelAngleEnabled;
    }

    public void setMaintainSwivelAngleCorrection(double maintainSwivelAngleCorrection) {
        this.maintainSwivelAngleCorrection = maintainSwivelAngleCorrection;
    }
}
