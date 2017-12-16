package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;
import static java.lang.Thread.sleep;


/**
 * Created by RAMSEY on 10/31/2017.
 */

/*
To activate turntable and arm motors, uncomment lines:
43-44
92-93
100-101
112-113
 */

public class Robot {
    public LinearOpMode l;
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightFrontDrive = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  linearLift      = null;
    public Servo    jewel           = null;
    public CRServo    leftSpin      = null;
    public CRServo    rightSpin     = null;
    public Servo    leftArm         = null;
    public Servo    rightArm        = null;
//    public Servo    relicGrip       = null;
//    public Servo    relicTilt       = null;
    public Telemetry telemetry;
    I2cDeviceSynch pixyCam;
    // The IMU sensor object
    BNO055IMU imu;

    //sensors
    public I2cDevice cryptoColorSensor;
    public I2cDeviceSynchImpl cryptoColorSensor2;
//    public ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    public ColorSensor colorSensor;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public VuforiaLocalizer vuforia;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    public final static double JEWEL_HOME = 1;
    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Boolean isAutonomous) {
        // save reference to HW Map
        hwMap = hardwareMap;
        this.telemetry  = telemetry;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive  = hwMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hwMap.get(DcMotor.class, "right_rear_drive");
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        linearLift=hwMap.get(DcMotor.class, "linear_lift");


//        turnTable = hwMap.get(DcMotor.class, "turntable");
//        arm = hwMap.get(DcMotor.class, "arm");

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        linearLift.setPower(0);
//        turnTable.setPower(0);
//        arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        if(isAutonomous) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        jewel = hwMap.get(Servo.class, "jewel");
        leftSpin = hwMap.get(CRServo.class, "left_spin");
        rightSpin = hwMap.get(CRServo.class, "right_spin");
        leftArm = hwMap.get(Servo.class, "left_arm");
        rightArm = hwMap.get(Servo.class, "right_arm");
//        relicGrip = hwMap.get(Servo.class, "relic_grip");
//        relicTilt = hwMap.get(Servo.class, "relic_tilt");

            if (isAutonomous) {
                jewel.setPosition(JEWEL_HOME);
                //Define Sensors
                pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");
                // State used for updating telemetry
                BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();
                imuparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imuparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                imuparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                imuparameters.loggingEnabled = true;
                imuparameters.loggingTag = "IMU";

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hwMap.get(BNO055IMU.class, "imu");
                imu.initialize(imuparameters);

                //Initialize VuForia
                int cameraMonitorViewId;

                cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
                VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

                // OR...  Do Not Activate the Camera Monitor View, to save power
                // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

                parameters.vuforiaLicenseKey = "AQqvrk//////AAAAGb0DGFsQf00Bg/3UWm/HGWNTXpC7ZHleP/bABJs7BCQiTDJAZaUUpD+oQMkuPRWclkwoQYSIfeaySqx5uaXNLfn16EPjQA4ZoiAUsx7SVFx9nJ0K00fhT1dA8bgtrsZF+KI0czmSWIB49V6qGwjUOp6+AUea7HEhpjqlux4m/ecVadObRqlc/bJdghlrHu0Z+Pu8m/CZjvEjxQjJmetQG8xyd3tucZwVK1gE3Wn2oNTS1179i/An4aKXIXk9BotxMYrrr/OkFZ2deJ4Z8h72RURuc+5DslTgJQZGq7rnT0k3dUirrbR9MTKCcAuMtggsWy2IyX0fC0StdCKOQbp0GUXZJEpnwtDCCDngCbPxNWv9";

                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

    }
    public void driveStraight(double wheelPower, long seconds) throws InterruptedException {
        leftFrontDrive.setPower(wheelPower);
        leftRearDrive.setPower(wheelPower);
        rightFrontDrive.setPower(wheelPower);
        rightRearDrive.setPower(wheelPower);
        sleep(seconds*1000);
        stopMotors();
    }
    public void driveStraightWhile(double wheelPower){
        leftFrontDrive.setPower(wheelPower);
        leftRearDrive.setPower(wheelPower);
        rightFrontDrive.setPower(wheelPower);
        rightRearDrive.setPower(wheelPower);
    }
    public void stopMotors(){
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
    public void turnRight(double wheelPower){
        leftFrontDrive.setPower(wheelPower);
        leftRearDrive.setPower(wheelPower);
        rightFrontDrive.setPower(-wheelPower);
        rightRearDrive.setPower(-wheelPower);
    }
//    public void turnWithGyro(LinearOpMode lom, double wheelPower, String direction, double degrees){
////        l = lom;
////        double currentHeading;
////        currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
////        degrees = degrees *.90;
////        if (direction.equals("right")) {
////            degrees = -degrees;
////            while (currentHeading >= degrees && l.opModeIsActive()) {
////                leftFrontDrive.setPower(wheelPower);
////                leftRearDrive.setPower(wheelPower);
////                rightFrontDrive.setPower(-wheelPower);
////                rightRearDrive.setPower(-wheelPower);
////                currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
////                telemetry.addData("CURRENT HEADING",currentHeading);
////                telemetry.update();
////            }
////        } else if (direction.equals("left")) {
////            while (currentHeading <= degrees && l.opModeIsActive()) {
////                leftFrontDrive.setPower(-wheelPower);
////                leftRearDrive.setPower(-wheelPower);
////                rightFrontDrive.setPower(wheelPower);
////                rightRearDrive.setPower(wheelPower);
////                currentHeading = modernRoboticsI2cGyro.getIntegratedZValue();
////                telemetry.addData("CURRENT HEADING",currentHeading);
////                telemetry.update();
////            }
////
////        }
//    }
    public void turnWithIMU(LinearOpMode lom, double wheelPower, String direction, double heading) {
        l = lom;
        Orientation angles;
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        heading = heading * .90;
        if (direction.toLowerCase().equals("right")) {
            heading = -heading;
            while (currentHeading >= heading && l.opModeIsActive()) {
                leftFrontDrive.setPower(wheelPower);
                leftRearDrive.setPower(wheelPower);
                rightFrontDrive.setPower(-wheelPower);
                rightRearDrive.setPower(-wheelPower);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;
                telemetry.addData("CURRENT HEADING RIGHT", currentHeading);
                telemetry.update();
            }
        } else if (direction.toLowerCase().equals("left")) {
            while (currentHeading <= heading && l.opModeIsActive()) {
                leftFrontDrive.setPower(-wheelPower);
                leftRearDrive.setPower(-wheelPower);
                rightFrontDrive.setPower(wheelPower);
                rightRearDrive.setPower(wheelPower);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentHeading = angles.firstAngle;
                telemetry.addData("CURRENT HEADING", currentHeading);
                telemetry.update();
            }
        }
    }
    public void resetDriveEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runWithoutEncoders(){
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void imuDrive (LinearOpMode lom,double speed, double distance, double angle) {
        l = lom;
        double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest 40
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * 3.1415);
        double     P_DRIVE_COEFF           = 0.0375;     // Larger is more responsive, but also less stable

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        resetDriveEncoders();

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        newRightTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(newLeftTarget);
        leftRearDrive.setTargetPosition(newLeftTarget);
        rightFrontDrive.setTargetPosition(newRightTarget);
        rightRearDrive.setTargetPosition(newRightTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftRearDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightRearDrive.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && l.opModeIsActive())) {

            // adjust relative speed based on heading error.
            error = getError(l,angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftFrontDrive.setPower(leftSpeed);
            leftRearDrive.setPower(leftSpeed);
            rightFrontDrive.setPower(rightSpeed);
            rightRearDrive.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();

        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getError(LinearOpMode lom, double targetAngle) {
        l = lom;
        double robotError;
        Orientation angles;
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - currentHeading;
        while (robotError > 180 && l.opModeIsActive())  robotError -= 360;
        while (robotError <= -180 && l.opModeIsActive()) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public String vuMark(LinearOpMode lom,int distance, double speed, double angle) throws InterruptedException {
        double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest 40
        double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /  (WHEEL_DIAMETER_INCHES * 3.1415);
        double     P_DRIVE_COEFF           = 0.0375;     // Larger is more responsive, but also less stable

        int moveCounts = 0;
        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;
        int count = 0;
        String vuMarkString;
        l = lom;
        int     newLeftTarget;
        int     newRightTarget;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        resetDriveEncoders();

        //initialize vuforia reading
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark;

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        newRightTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(newLeftTarget);
        leftRearDrive.setTargetPosition(newLeftTarget);
        rightFrontDrive.setTargetPosition(newRightTarget);
        rightRearDrive.setTargetPosition(newRightTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        leftFrontDrive.setPower(speed);
        leftRearDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightRearDrive.setPower(speed);
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        // keep looping while we are still active, and BOTH motors are running.
        while ((leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && l.opModeIsActive())) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark.toString().equals("UNKNOWN")){
                speed = .1;
            } else {
                speed = .2;
            }

            // adjust relative speed based on heading error.
            error = getError(l,angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            leftFrontDrive.setPower(leftSpeed);
            leftRearDrive.setPower(leftSpeed);
            rightFrontDrive.setPower(rightSpeed);
            rightRearDrive.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData("VMLEFT",leftCount);
            telemetry.addData("VMCENTER",centerCount);
            telemetry.addData("VMRIGHT",rightCount);
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();

        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vuMarkString = vuMark.toString();

        return vuMarkString;
    }
    public String vuMark(LinearOpMode lom, double speed, double headingToTurnTo) throws InterruptedException {
        l = lom;
        String vuMarkString = "UNKNOWN";
        int count = 0;
        headingToTurnTo = headingToTurnTo * .90;
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark;

        turnWithIMU(l,speed,"left",headingToTurnTo);
        stopMotors();
        while (count < 25 && l.opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMarkString = vuMark.toString();
            count++;
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            sleep(100);
        }

        return vuMarkString;
    }
    public String vuMark(LinearOpMode lom, double seconds){
        l = lom;
        String vuMarkString = "UNKNOWN";
        int count = 0;
        double timeVumark = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark;
        timeVumark = timer.time();
        while (timeVumark < seconds && l.opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            vuMarkString = vuMark.toString();
            count++;
            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
            timeVumark = timer.time();
        }

        return vuMarkString;
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
//    public void setTurnTable(int position, double speed){
//        turnTable.setTargetPosition(position);
//        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//        turnTable.setPower(speed);
//    }

    public String whichIsLeft(LinearOpMode lom) throws InterruptedException{
        l = lom;
        String whichIsLeft = "UNKNOWN";
        double redBallX;
        double blueBallX;
        byte[] redBall;
        byte[] blueBall;
        int i = 0;
        int redBallLeftCount = 0;
        int blueBallLeftCount = 0;

        pixyCam.engage();
        sleep(500);
        while(l.opModeIsActive() && i < 3 ){
            redBall = pixyCam.read(0x51,5);
            redBallX = (0xff&redBall[1]);
            blueBall = pixyCam.read(0x52,5);
            blueBallX = (0xff&blueBall[1]);
            if (redBallX < blueBallX){
                redBallLeftCount++;
            } else if (redBallX > blueBallX){
                blueBallLeftCount++;
            }

            telemetry.addData("RED X VALUE", 0xff&redBall[1]);
            telemetry.addData("BLUE X VALUE", 0xff&blueBall[1]);
            telemetry.addData("RED COUNT",redBallLeftCount);
            telemetry.addData("BLUE COUNT", blueBallLeftCount);
            telemetry.update();
            sleep (500);
            i++;
        }
        if (redBallLeftCount > blueBallLeftCount){
            whichIsLeft = "RED";
        } else if (blueBallLeftCount > redBallLeftCount){
            whichIsLeft = "BLUE";
        } else {
            whichIsLeft = "UNKNOWN";
        }
        return whichIsLeft;
    }

    public void openArms(){
        //swing arms out
        leftArm.setPosition(.25);
        rightArm.setPosition(.75);
    }

    public void closeArms(){
        //squeeze block
        leftArm.setPosition(.05);
        rightArm.setPosition(.9);
    }
    public void pushBlockOut(){
        leftSpin.setPower(.8);
        rightSpin.setPower(-.8);
    }
    public void pullBlockIn(){
        leftSpin.setPower(-.8);
        rightSpin.setPower(.8);
    }


 public void scoreBlock(LinearOpMode lom, double cryptoHeading) throws InterruptedException {
     linearLift.setPower(.5);
     sleep(2000);
     imuDrive(lom, .1,4 , cryptoHeading);
     sleep(100);
     openArms();
     sleep(500);
     driveStraight(-.1,1);
     closeArms();
     sleep(100);
     driveStraight(.1,1);
     sleep(100);
     driveStraight(-.1,1);
 }
 public void getMoreBlocks(LinearOpMode lom) throws InterruptedException{
     imuDrive(lom,-.75, -24,90);
     stopMotors();
     sleep(100);
     turnWithIMU(lom,.5,"left",175);
     stopMotors();
     sleep(1000);
     turnWithIMU(lom,.5,"left",-90);
     stopMotors();
     sleep(100);
     imuDrive(lom,.75, 20,-90);
     leftArm.setPosition(.25);
     rightArm.setPosition(.9);
     leftSpin.setPower(1);
     rightSpin.setPower(-1);
     imuDrive(lom,.5, 10,-90);
     stopMotors();
     closeArms();
     leftSpin.setPower(1);
     rightSpin.setPower(-1);
     linearLift.setPower(-1);
     sleep(500);
     linearLift.setPower(0);
     turnWithIMU(lom,.5,"left",90);
     stopMotors();
     sleep(100);
     imuDrive(lom,.75,54,90);
     stopMotors();
     sleep(100);
     leftSpin.setPower(0);
     rightSpin.setPower(0);
 }
}
