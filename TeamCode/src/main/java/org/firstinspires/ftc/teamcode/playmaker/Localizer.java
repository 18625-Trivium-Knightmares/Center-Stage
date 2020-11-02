package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Localizer {

    private long latestAcquisitionTime = 0;
    private OpenGLMatrix cameraMatrix;
    private List<VuforiaTrackable> vuforiaTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTransform lastVuforiaTransform;
    private Orientation lastRawIMUOrientation;
    private Orientation lastIMUOrientation;
    private Double imuToWorldRotation;

    // Vuforia Constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    public void setLatestAcquisitionTime() {
        this.latestAcquisitionTime = System.nanoTime();
    }


    /**
     * Return the robot's estimated location on the field.
     * X: the axis running from the audience view (negative) to the goals (positive)
     * Y: the axis running from red alliance (negative) to the blue alliance (positive)
     * Z: height of the robot off ground, though not relevant for this game.
     * @return the estimated Position
     */

    public Position estimatePosition() {

        if (lastVuforiaTransform != null) {
            VectorF translation = lastVuforiaTransform.transform.getTranslation();
            float x = translation.get(0);
            float y = translation.get(1);
            float z = translation.get(2);
            Position position = new Position(DistanceUnit.MM, x, y, z, System.currentTimeMillis());
            return position;
        }
        return null;
    }

    /**
     * Estimate the robot's orientation on the field.
     * First Angle: Roll
     * Second Angle: Pitch
     * Third Angle: Heading, this is the only value you'll really care about
     * @return
     */

    enum OrientationSource {
        VUFORIA,
        IMU,
        OTHER
    }

    enum PositionSource {
        VUFORIA,
        ENCODERS,
        OTHER
    }

    public class EstimatedPosition {
        public PositionSource source;
        public Position position;
        public EstimatedPosition(PositionSource soruce, Position position) {
            this.source = source;
            this.position = position;

        }
    }

    public class EstimatedOrientation {
        public OrientationSource source;
        public Orientation orientation;
        public EstimatedOrientation(OrientationSource source, Orientation orientation) {
            this.source = source;
            this.orientation = orientation;
        }
    }

    public EstimatedOrientation estimateOrientation() {
        if (lastVuforiaTransform != null) {
            Orientation rotation = Orientation.getOrientation(lastVuforiaTransform.transform, EXTRINSIC, XYZ, DEGREES);
            return new EstimatedOrientation(OrientationSource.VUFORIA, rotation);
        } else if (lastIMUOrientation != null) {
            return new EstimatedOrientation(OrientationSource.IMU, lastIMUOrientation);
        }
        return null;
    } //meeep

    public void telemetry(Telemetry telemetry) {
        Position position = estimatePosition();
        EstimatedOrientation orientation = estimateOrientation();
        if (position != null) {
            telemetry.addData("Loc. Position", String.format("%.1f, %.1f, %.1f", position.x, position.y, position.z));
        } else {
            telemetry.addData("Loc. Position", "unknown");
        }

        if (orientation != null) {
            telemetry.addData("Loc. Orientation Source", orientation.source.toString());
            Orientation rotation = orientation.orientation;
            telemetry.addData("Loc. Orientation", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Loc. Orientation", "unknown");
        }

        if (lastRawIMUOrientation != null) {
            Orientation rotation = lastRawIMUOrientation;
            telemetry.addData("Raw IMU. Orientation", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Raw IMU. Orientation", "unknown");
        }

    }



    /**
     * Sets the camera matrix for navigation.
     * @param hardware Robot Hardware being used
     * @param newCameraMatrix The new camera matrix to be applied
     */
    public void setCameraMatrix(RobotHardware hardware, OpenGLMatrix newCameraMatrix) {
        this.cameraMatrix = newCameraMatrix;
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setPhoneInformation(this.cameraMatrix, hardware.vuforiaParameters.cameraDirection);
        }
    }

    /**
     * Sets the camera matrix for navigation using given offset and rotation.
     * @param hardware Robot Hardware being used
     * @param cameraOffset Offset of the camera relative to the robot's center (0,0,0)
     * @param cameraRotation Rotation of the camera relative to the robot's forward direction
     */
    public void setCameraMatrix(RobotHardware hardware, Position cameraOffset, Orientation cameraRotation) {
        this.cameraMatrix = OpenGLMatrix.translation((float) cameraOffset.x, (float) cameraOffset.y, (float) cameraOffset.z);
        this.cameraMatrix = this.cameraMatrix.multiplied(cameraRotation.getRotationMatrix());
        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            listener.setPhoneInformation(this.cameraMatrix, hardware.vuforiaParameters.cameraDirection);
        }
    }

    /**
     * Load the Ultimate Goal Trackables. Largely copied from ConceptVuforiaUltimteGoalNavigationWebcam
     */
    public void loadUltimateGoalTrackables(RobotHardware hardware) {
        VuforiaTrackables targetsUltimateGoal = hardware.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        vuforiaTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        targetsUltimateGoal.activate();
    }

    public void updateRobotTransform(RobotHardware hardware, BNO055IMU imu) {
        this.setLatestAcquisitionTime();
        this.updateLocationWithVuforia(hardware);
        if (imu != null) {
            this.updateIMUOrientation(imu);
        }

    }
    public void updateLocationWithVuforia(RobotHardware hardware) {
        // Clear the currently saved vuforia transform
        lastVuforiaTransform = null;

        for (VuforiaTrackable trackable : vuforiaTrackables) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) trackable.getListener();
            if (listener.isVisible()) {
                VectorF translation = listener.getVuforiaCameraFromTarget().getTranslation();
                //hardware.telemetry.addData("Localizer Visible Target:", trackable.getName());
                //hardware.telemetry.addData("Localizer Visible Target Rel Camera", String.format("%.1f, %.1f, %.1f", translation.get(0), translation.get(1), translation.get(2)));
                OpenGLMatrix robotTransform = listener.getRobotLocation();
                if (robotTransform != null) {
                    lastVuforiaTransform = new VuforiaTransform(robotTransform);
                }
            }
        }
    }

    /**
     * This will attempt to find the differences of orientation between the field-centric system and
     * the IMU reference system.
     * @return Whether calibration was successful
     */

    public boolean attemptIMUToWorldCalibration(BNO055IMU imu) {
        if (lastVuforiaTransform != null && imu.isGyroCalibrated()) {
            Orientation vuforiaRotation = Orientation.getOrientation(lastVuforiaTransform.transform, EXTRINSIC, XYZ, DEGREES);
            Orientation imuRotation = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES);
            imuToWorldRotation = Localizer.angularDifference(vuforiaRotation.thirdAngle, imuRotation.thirdAngle);
            return true;
        }
        return false;
    }

    public Double getImuToWorldRotation() {
        return this.imuToWorldRotation;
    }

    public void updateIMUOrientation(BNO055IMU imu) {
        Orientation imuOrientation = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES);
        this.lastRawIMUOrientation = imuOrientation;
        if (this.imuToWorldRotation != null) {
            double imuHeading = imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).thirdAngle;
            double worldHeading = Localizer.headingWrapDegrees(imuHeading + this.imuToWorldRotation);
            this.lastIMUOrientation = new Orientation(EXTRINSIC, XYZ, DEGREES, imuOrientation.firstAngle, imuOrientation.secondAngle, (float) worldHeading, System.nanoTime());
        }
    }


    public class VuforiaTransform {
        public long acquisitionTime;
        public OpenGLMatrix transform;

        public VuforiaTransform(OpenGLMatrix transform) {
            this.acquisitionTime = System.nanoTime();
            this.transform = transform;

        }
    }

    /**
     * Compute the XY distance between two Positions
     * @param a Position 1
     * @param b Position 2
     * @return XY Distance
     */
    public static double distance(Position a, Position b, DistanceUnit unit) {
        Position unit_a = a.toUnit(unit);
        Position unit_b = b.toUnit(unit);
        return Math.hypot(unit_b.x - unit_a.x, unit_b.y - unit_a.y);
    }

    /**
     *
     * @param start Starting angle
     * @param end End angle
     * @return An angle from -180 to 180, where positive angles indicate a rotation to the left and vice versa.
     */
    public static double angularDifference(double start, double end) {
        return (start - end + 180) % 360 - 180;
    }

    public static int mod(int x, int n) {
        return (x % n) - (x < 0 ? n : 0);
    }

    public static double mod(double x, double n) {
        return (x % n) - (x < 0 ? n : 0);
    }

    public static float mod(float x, float n) {
        return (x % n) - (x < 0 ? n : 0);
    }

    public static long mod(long x, long n) {
        return (x % n) - (x < 0 ? n : 0);
    }

    public static double headingWrapDegrees(double angle) {
        return Localizer.mod(angle + 180, 360) - 180;
    }

    public static double headingWrapRadians(double angle) {
        return Localizer.mod(angle + Math.PI, 2*Math.PI) - Math.PI;
    }

    public static double atan2(Position a, Position b) {
        Position unit_a = a.toUnit(DistanceUnit.CM);
        Position unit_b = b.toUnit(DistanceUnit.CM);
        return Math.toDegrees(Math.atan2((unit_b.x-unit_a.x),-(unit_b.y-unit_a.y)));
    }

}
