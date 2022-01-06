package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.Base64;

public class Robot extends java.lang.Thread {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //private static final int TICKS_PER_ROTATION = 1440; //Tetrix motor specific
    private static final double TICKS_PER_ROTATION = 537.6; //Gobilda 1150 RMP motor specific
    private static final double WHEEL_DIAMETER = 4.5; //Wheel diameter in inches
    private String TAG = "FTC";

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;
    private DcMotorEx inslide;
    private DcMotorEx outslide;
    private DcMotorEx inflip;
    private DcMotorEx outflip;
    private CRServo intake;
    private CRServo outtake;
    private CRServo carousel;
    // The IMU sensor object
    private BNO055IMU imu;

    private final int tollerance = 15;

    // State used for updating telemetry
    private Orientation angles;
    private PIDController pidRotate, pidDrive;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;

    public boolean isTeleOp = true;
    private boolean DEBUG_DEBUG = true;
    private boolean DEBUG_INFO = false;

    private long movementFactor = 1;
    private double turnFactor = 4.4;
    private double leftStrafeFactor = 1.2;
    private double rightStrafeFactor = 1.2;

    Robot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;

        initDevices();
    }

    /* Calculate Drivetrain PID cofficients */

    private final int motorFLMaxSpeed = 2960;
    double motorFLF = 32767 / (double) motorFLMaxSpeed;
    double motorFLP = 0.1 * motorFLF;
    double mototFLI = 0.1 * motorFLP;

    private final int motorFRMaxSpeed = 3120;
    double motorFRF = 32767 / (double) motorFRMaxSpeed;
    double motorFRP = 0.1 * motorFRF;
    double mototFRI = 0.1 * motorFRP;

    private final int motorBLMaxSpeed = 3100;
    double motorBLF = 32767 / (double) motorBLMaxSpeed;
    double motorBLP = 0.1 * motorBLF;
    double mototBLI = 0.1 * motorBLP;

    private final int motorBRMaxSpeed = 2920;
    double motorBRF = 32767 / (double) motorBRMaxSpeed;
    double motorBRP = 0.1 * motorBRF;
    double mototBRI = 0.1 * motorBRP;
    boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
    double  targetRange     = 0;        // Distance from camera to target in Inches
    double  targetX         = 0;        // X Distance
    double  targetY         = 0;        // Y Distance
    double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
    double  drive           = 0;        // Desired forward power (-1 to +1)
    double  turn            = 0;        // Desired turning power (-1 to +1)

    private static final String VUFORIA_KEY = "AV8zEej/////AAABmVo2vNWmMUkDlkTs5x1GOThRP0cGar67mBpbcCIIz/YtoOvVynNRmJv/0f9Jhr9zYd+f6FtI0tYHqag2teC5GXiKrNM/Jl7FNyNGCvO9zVIrblYF7genK1FVH3X6/kQUrs0vnzd89M0uSAljx0mAcgMEEUiNOUHh2Fd7IOgjlnh9FiB+cJ8bu/3WeKDxnDdqx6JI5BlQ4w7YW+3X2icSRDRlvE4hhuW1VM1BTPQgds7OtHKqUn4Z5w1Wqg/dWiOHxYTww28PVeg3ae4c2l8FUtE65jr2qQdQNc+DMLDgnJ0fUi9Ww28OK/aNrQQnHU97TnUgjLgCTlV7RXpfut5mZWXbWvO6wA6GGkm3fAIQ2IPL";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";


    private void initDeviceCore() throws Exception {

        telemetry.addData("Please wait", "In function init devices");
        telemetry.update();

        //Wheels
        inslide = hardwareMap.get(DcMotorEx.class, "inslide");
        outslide = hardwareMap.get(DcMotorEx.class, "outslide");
        Motor_FL = hardwareMap.get(DcMotorEx.class, "motor_fl");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "motor_fr");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "motor_br");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "motor_bl");
        inflip = hardwareMap.get(DcMotorEx.class, "inflip");
        intake = hardwareMap.get(CRServo.class, "intake");
        outtake = hardwareMap.get(CRServo.class, "outtake");
        outflip = hardwareMap.get(DcMotorEx.class, "outflip");
        carousel = hardwareMap.get(CRServo.class, "carousel");

       /*Motor_FR.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FR.setPositionPIDFCoefficients(5.0);
        Motor_FL.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FL.setPositionPIDFCoefficients(5.0);
        Motor_BR.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BR.setPositionPIDFCoefficients(5.0);
        Motor_BL.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BL.setPositionPIDFCoefficients(5.0);*/

        Log.i(TAG, "Motor FR Cofficients: P: " + motorFRP + " I: " + mototFRI + " F: " + motorFRF);
        Log.i(TAG, "Motor FL Cofficients: P: " + motorFLP + " I: " + mototFLI + " F: " + motorFLF);
        Log.i(TAG, "Motor BR Cofficients: P: " + motorBRP + " I: " + mototBRI + " F: " + motorBRF);
        Log.i(TAG, "Motor BL Cofficients: P: " + motorBLP + " I: " + mototBLI + " F: " + motorBLF);

        Motor_FR.setVelocityPIDFCoefficients(motorFRP, mototFRI, 0, motorFRF);
        Motor_FR.setPositionPIDFCoefficients(8.2);

        Motor_FL.setVelocityPIDFCoefficients(motorFLP, mototFLI, 0, motorFLF);
        Motor_FL.setPositionPIDFCoefficients(8.0);

        Motor_BR.setVelocityPIDFCoefficients(motorBRP, mototBRI, 0, motorBRF);
        Motor_BR.setPositionPIDFCoefficients(8.2);

        Motor_BL.setVelocityPIDFCoefficients(motorBLP, mototBLI, 0, motorBLF);
        Motor_BL.setPositionPIDFCoefficients(8.0);

        Motor_FL.setTargetPositionTolerance(15);
        Motor_FR.setTargetPositionTolerance(15);
        Motor_BL.setTargetPositionTolerance(15);
        Motor_BR.setTargetPositionTolerance(15);


        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
//        parameters.useExtendedTracking = false;
//
//        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json";
        parametersIMU.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.0099, .0001, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Start Orientation First : " + angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    private void initDevices() {
        ElapsedTime mRuntime;
        mRuntime = new ElapsedTime();
        mRuntime.reset();

        try {
            initDeviceCore();
        } catch (Exception e) {
            telemetry.addData("Exception", "In function init devices" + e);
            telemetry.update();
            try {
                sleep(10000);
            } catch (Exception e1) {
            }

        }

    }

    private void pause(int milliSec) {
        try {
            sleep(milliSec);
        } catch (Exception e) {
        }
    }

    // This function takes input distance in inches and will return Motor ticks needed
    // to travel that distance based on wheel diameter
    private int DistanceToTick(double distance) {
        // Log.i(TAG, "Enter FUNC: DistanceToTick");

        double circumference = WHEEL_DIAMETER * 3.14;
        double num_rotation = distance / circumference;
        int encoder_ticks = (int) (num_rotation * TICKS_PER_ROTATION);

        //       Log.i(TAG,"Rotation Needed : " + num_rotation);
        //if (DEBUG_INFO) {
        Log.i(TAG, "Ticks Needed : " + encoder_ticks);
        //  Log.i(TAG, "Exit FUNC: DistanceToTick");
        //}

        return (encoder_ticks);
    }

    // This function takes input Angle (in degrees)  and it will return Motor ticks needed
    // to make that Turn2
    private int AngleToTick(double angle) {
        Log.i(TAG, "Enter FUNC: AngleToTick");

        int encoder_ticks = (int) ((java.lang.Math.abs(angle) * TICKS_PER_ROTATION) / 360);

        Log.i(TAG, "Ticks needed for Angle : " + encoder_ticks);
        Log.i(TAG, "Exit FUNC: AngleToTick");

        return (encoder_ticks);
    }

    boolean drivetrainBusy(int ticks) {
        int avg = (java.lang.Math.abs(Motor_FL.getCurrentPosition())
                + java.lang.Math.abs(Motor_FR.getCurrentPosition())
                + java.lang.Math.abs(Motor_BL.getCurrentPosition())
                + java.lang.Math.abs(Motor_BR.getCurrentPosition())) / 4;
        if ((avg >= (ticks - tollerance)) || (avg <= (ticks + tollerance))) {
            return false;
        }
        return true;
    }

    public void setMotorPowers(double powerFL, double powerFR, double powerBL, double powerBR) {
        Motor_FL.setPower(powerFL);
        Motor_FR.setPower(powerFR);
        Motor_BL.setPower(powerBL);
        Motor_BR.setPower(powerBR);
    }

    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
        // Move forward to specific distance in inches, with power (0 to 1)
    public void moveBackwardToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveBackwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(0.99 * power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        while (Motor_FL.isBusy() || Motor_BL.isBusy() || Motor_FR.isBusy() || Motor_BR.isBusy()) {
        while (Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Backward", "Moving");
            telemetry.update();
        }

        Log.i(TAG, "Finished with target " + ticks + " ticks");

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveBackwardToPosition");
        }
    }

    // Move backward to specific distance in inches, with power (0 to 1)
    public void moveForwardToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveForwardToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition(ticks);
        Motor_FR.setTargetPosition((-1) * ticks);
        Motor_BR.setTargetPosition((-1) * ticks);
        Motor_BL.setTargetPosition(ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(0.99 * power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for them to reach to the position
        // while ((Motor_FL.isBusy() && Motor_BL.isBusy()) || (Motor_FR.isBusy() && Motor_BR.isBusy())){
//        while (Motor_FL.isBusy() || Motor_BL.isBusy() || Motor_FR.isBusy() || Motor_BR.isBusy()) {
        while (Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Forward", "Moving");
            telemetry.update();
        }


        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveForwardToPosition");
        }
    }

    // Move Left to specific distance in inches, with power (0 to 1)
    public void moveLeftToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveRightToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);
        ticks = (int) (ticks * leftStrafeFactor);
        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition(ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition((-1) * ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(1.3 * power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Wait for them to reach to the position
        // while ((Motor_FR.isBusy() && Motor_BL.isBusy()) || (Motor_FL.isBusy() && Motor_BR.isBusy())){
        while (Motor_FL.isBusy()) {

            if ((System.currentTimeMillis() - startTime) > 2000) {
                break;
            }

            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Right", "Moving");
            telemetry.update();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveRightToPosition");
        }
    }

    public void moveLeftToPositionWithFeedback(double power, double distance) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();
        int i = 0;
        int totalTicks = 0;
        double flPower = power;
        double frPower = power;
        double brPower = power;
        double blPower = power;

        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);
        ticks = (int) (ticks * rightStrafeFactor);
        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition(ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition((-1) * ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(flPower);
        Motor_FR.setPower(frPower);
        Motor_BR.setPower(brPower);
        Motor_BL.setPower(blPower);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Wait for them to reach to the position
        // while ((Motor_FL.isBusy() && Motor_BR.isBusy()) || (Motor_FR.isBusy() && Motor_BL.isBusy())){
        while ((Motor_FL.isBusy()
                && Motor_FR.isBusy()
                && Motor_BL.isBusy()
                && Motor_BR.isBusy())) {

            i++;

            if ((i%5) == 0 ){
                totalTicks = Math.abs(Motor_FL.getCurrentPosition()) +
                        Math.abs(Motor_FR.getCurrentPosition())+
                        Math.abs(Motor_BR.getCurrentPosition()) +
                        Math.abs(Motor_BL.getCurrentPosition());

                if (Math.abs(Motor_FL.getCurrentPosition()) > (totalTicks/4)){
                    Motor_FL.setPower(flPower * 0.9);
                }
                else if (Math.abs(Motor_FL.getCurrentPosition()) < (totalTicks/4)){
                    flPower = flPower * 1.1;
                    if (flPower > 1)
                        flPower = 1;
                    Motor_FL.setPower(flPower);
                }

                if (Math.abs(Motor_FR.getCurrentPosition()) > (totalTicks/4)){
                    Motor_FR.setPower(frPower * 0.9);
                }
                else if (Math.abs(Motor_FR.getCurrentPosition()) < (totalTicks/4)){
                    frPower = frPower * 1.1;
                    if (frPower > 1)
                        frPower = 1;
                    Motor_FR.setPower(frPower);
                }

                if (Math.abs(Motor_BR.getCurrentPosition()) > (totalTicks/4)){
                    Motor_BR.setPower(brPower * 0.9);
                }
                else if (Math.abs(Motor_BR.getCurrentPosition()) < (totalTicks/4)){
                    brPower = brPower * 1.1;
                    if (brPower > 1)
                        brPower = 1;
                    Motor_FR.setPower(brPower);
                }

                if (Math.abs(Motor_BL.getCurrentPosition()) > (totalTicks/4)){
                    Motor_BL.setPower(blPower * 0.9);
                }
                else if (Math.abs(Motor_BL.getCurrentPosition()) < (totalTicks/4)){
                    blPower = blPower * 1.1;
                    if (blPower > 1)
                        blPower = 1;
                    Motor_FR.setPower(blPower);
                }
            }

            if ((System.currentTimeMillis() - startTime) > 1000){
                break;
            }

            if (DEBUG_DEBUG) {
                Log.i(TAG, "flspeed : " + flPower);
                Log.i(TAG, "frspeed : " + frPower);
                Log.i(TAG, "brspeed : " + brPower);
                Log.i(TAG, "blspeed : " + blPower);
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Left", "Moving");
            telemetry.update();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveLeftToPosition");
        }
    }

    // Move Right to specific distance in inches, with power (0 to 1)
    public void moveRightToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);
        ticks = (int) (ticks * rightStrafeFactor);
        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition((-1) * ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition(ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(1.3 * power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Wait for them to reach to the position
        // while ((Motor_FL.isBusy() && Motor_BR.isBusy()) || (Motor_FR.isBusy() && Motor_BL.isBusy())){
        while (Motor_FL.isBusy()) {

            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }

            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Left", "Moving");
            telemetry.update();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveLeftToPosition");
        }
    }

    public void moveRightToPositionWithFeedback(double power, double distance) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();
        int i = 0;
        int totalTicks = 0;
        double flPower = power * 0.7;
        double frPower = power;
        double brPower = power * 1.15;
        double blPower = power;

        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);
        ticks = (int) (ticks * rightStrafeFactor);
        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition((-1) * ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition(ticks);

        //Set power of all motors
        Motor_FL.setPower(flPower);
        Motor_FR.setPower(frPower);
        Motor_BR.setPower(brPower);
        Motor_BL.setPower(blPower);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double UPGRADE = 1.01;
        double DOWNGRADE = 0.995;
        double VARIATION_UP = 1.1;
        double VARIATION_DOWN = 0.9;
        //Wait for them to reach to the position
        // while ((Motor_FL.isBusy() && Motor_BR.isBusy()) || (Motor_FR.isBusy() && Motor_BL.isBusy())){
        while ((Motor_FL.isBusy()
                && Motor_FR.isBusy()
                && Motor_BL.isBusy()
                && Motor_BR.isBusy())) {

            i++;

            if ((i%1) == 0 && i > 5){
                totalTicks = -1*Motor_FL.getCurrentPosition() +
                        -1*Motor_FR.getCurrentPosition() +
                       Motor_BR.getCurrentPosition() +
                        Motor_BL.getCurrentPosition();

                if (Math.abs(Motor_FL.getCurrentPosition()) > (totalTicks/4) * VARIATION_UP){
                    flPower = flPower * DOWNGRADE;
                    Motor_FL.setPower(flPower);
                }
                else if (Math.abs(Motor_FL.getCurrentPosition()) < (totalTicks/4) * VARIATION_DOWN){
                    flPower = flPower * UPGRADE;
                    if (flPower > 1)
                        flPower = 1;
                    Motor_FL.setPower(flPower);
                }

                if (Math.abs(Motor_FR.getCurrentPosition()) > (totalTicks/4) * VARIATION_UP){
                    frPower = frPower * DOWNGRADE;
                    Motor_FR.setPower(frPower);
                }
                else if (Math.abs(Motor_FR.getCurrentPosition()) < (totalTicks/4) * VARIATION_DOWN){
                    frPower = frPower * UPGRADE;
                    if (frPower > 1)
                        frPower = 1;
                    Motor_FR.setPower(frPower);
                }

                if (Math.abs(Motor_BR.getCurrentPosition()) > (totalTicks/4) * VARIATION_UP){
                    brPower = brPower * DOWNGRADE;
                    Motor_BR.setPower(brPower);
                }
                else if (Math.abs(Motor_BR.getCurrentPosition()) < (totalTicks/4) * VARIATION_DOWN){
                    brPower = brPower * UPGRADE;
                    if (brPower > 1)
                        brPower = 1;
                    Motor_FR.setPower(brPower);
                }

                if (Math.abs(Motor_BL.getCurrentPosition()) > (totalTicks/4) * VARIATION_UP){
                    blPower = blPower * DOWNGRADE;
                    Motor_BL.setPower(blPower);
                }
                else if (Math.abs(Motor_BL.getCurrentPosition()) < (totalTicks/4) * VARIATION_DOWN){
                    blPower = blPower * UPGRADE;
                    if (blPower > 1)
                        blPower = 1;
                    Motor_FR.setPower(blPower);
                }
                if (DEBUG_DEBUG) {

                    Log.i(TAG, "flspeed : " + flPower + ", ticks: " + Motor_FL.getCurrentPosition());
                    Log.i(TAG, "frspeed : " + frPower + ", ticks: " + Motor_FR.getCurrentPosition());
                    Log.i(TAG, "brspeed : " + brPower + ", ticks: " + Motor_BR.getCurrentPosition());
                    Log.i(TAG, "blspeed : " + blPower + ", ticks: " + Motor_BL.getCurrentPosition());
                    totalTicks = -1*Motor_FL.getCurrentPosition() +
                            -1*Motor_FR.getCurrentPosition() +
                            Motor_BR.getCurrentPosition() +
                            Motor_BL.getCurrentPosition();
                    Log.i(TAG, "i : " + i + ", average ticks: " + totalTicks/4 + "\n \n");
                }
            }

            if ((System.currentTimeMillis() - startTime) > 2500){
                break;
            }

            //Waiting for Robot to travel the distance
            telemetry.addData("Left", "Moving");
            telemetry.update();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveLeftToPosition");
        }
    }

    /*****************************************************************************/
    /* Section:      Move For specific time functions                            */
    /*                                                                           */
    /* Purpose:    Used if constant speed is needed                              */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     time          - Time in MilliSeconds                    */
    /*                                                                           */

    /*****************************************************************************/
    // Move forward for specific time in milliseconds, with power (0 to 1)
    public void moveBackwardForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveBackwardForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set power of all motors
        Motor_FL.setPower((-1) * power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower((-1) * power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveBackwardForTime");
    }

    public void moveForwardForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveForwardForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower(power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveForwardForTime");
    }

    public void moveRightForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveLeftForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set power of all motors
        Motor_FL.setPower((-1) * (0.92) * power);
        Motor_FR.setPower((-1) * (0.92) * power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);


        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveLeftForTime");
    }

    public void moveLeftForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveRightForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set power of all motors
        Motor_FL.setPower((0.92) * power);
        Motor_FR.setPower((0.92) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1) * power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveRightForTime");
    }

    public void turnForTime(double power, int time, boolean speed, int orientation) {
        Log.i(TAG, "Enter Function: turnForTime Power : " + power + " and time : " + time + "Speed : " + speed + "orientation : " + orientation);

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set power of all motors
        Motor_FL.setPower(orientation * power);
        Motor_FR.setPower(orientation * power);
        Motor_BR.setPower(orientation * power);
        Motor_BL.setPower(orientation * power);

       try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }

    public void turnWithPower(double power) {
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motor_FL.setPower(0.8 * power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(0.8 * power);
        Motor_BL.setPower(power);

    }
    public void arcTurn(double power, int time){
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(-0.1 * power);
        Motor_BL.setPower(-0.1*power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }
    public void arcTurnTwo(double power, int time){
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Motor_FL.setPower(-0.1*power);
        Motor_FR.setPower(-0.1*power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }
    public void turnOff() {
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }


    public void moveF(double power, long distance) {
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Motor_FL.setPower(power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower(power);
        /*try {
            sleep(distance * movementFactor);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        telemetry.addData("Direction", "Backward");
        telemetry.update();
        if (isTeleOp == false) pause(250);*/
    }

    public void moveB(double power, long distance) {
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Motor_FL.setPower((-1) * power); //FL
        Motor_FR.setPower(power); //FR
        Motor_BR.setPower(power); //BR
        Motor_BL.setPower((-1) * power); //BL
        /*try {
            sleep(distance * movementFactor);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        telemetry.addData("Direction", "Backward");
        telemetry.update();
        if (isTeleOp == false) pause(250);*/
    }

    public void moveR(double power, long distance) {
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Motor_FL.setPower((1) * power);
        Motor_FR.setPower((1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1.3) * power);
        /*try {
            sleep(distance * movementFactor);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        telemetry.addData("Direction", "Right");
        telemetry.update();
        if (isTeleOp == false) pause(250);*/
    }

    public void moveL(double power, long distance) {
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Motor_FL.setPower((-1) * (1) * power);
        Motor_FR.setPower((-1) * (1) * power);
        Motor_BR.setPower(power * (1));
        Motor_BL.setPower(power * (1.3));

        /*try {
            sleep(distance * movementFactor);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        telemetry.addData("Direction", "Left");
        telemetry.update();
        if (isTeleOp == false) pause(250);*/
    }

    public void slowTurn(double angle) {
        Log.i(TAG, "Enter Function slowTurn Angle: " + angle);
        int sleepTime = java.lang.Math.abs((int) (angle * turnFactor));

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (angle < 0) {
            Motor_FL.setPower(0.5);
            Motor_FR.setPower(0.5);
            Motor_BR.setPower(0.5);
            Motor_BL.setPower(0.5);
            try {
                Log.i(TAG, "SlowTurn Sleep: " + sleepTime);
                sleep(sleepTime);
            } catch (Exception e) {
                e.printStackTrace();
            }
            Motor_FL.setPower(0);
            Motor_FR.setPower(0);
            Motor_BR.setPower(0);
            Motor_BL.setPower(0);
        } else {
            Motor_FL.setPower(-0.5);
            Motor_FR.setPower(-0.5);
            Motor_BR.setPower(-0.5);
            Motor_BL.setPower(-0.5);
            try {
                Log.i(TAG, "SlowTurn Sleep: " + sleepTime);
                sleep(sleepTime);
            } catch (Exception e) {
                e.printStackTrace();
            }
            Motor_FL.setPower(0);
            Motor_FR.setPower(0);
            Motor_BR.setPower(0);
            Motor_BL.setPower(0);
        }
        Log.i(TAG, "Exit Function slowTurn");
    }


    public void tolerance() {
        int tolerance1;
        tolerance1 = Motor_FL.getTargetPositionTolerance();
        Log.i(TAG, "" + tolerance1);

    }


    public void fixOrientation(double degree) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Current Orientation First : " + angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle);
        double diff = degree - angles.firstAngle;
        if (diff < 10 || diff > -10) {
            slowTurn(diff);
        }

    }

    /* IMU based turn functions */


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateLeft(int degrees, double power) {
        Log.i(TAG, "Enter Function: rotate, Angle: " + degrees);

        double angle;
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
          /*  while (getAngle() == 0)
            {
                // set power to rotate.
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
                Log.i(TAG, "Function: rotate, Angle less then 0 Motor Power set to: " + power);
                try {
                    sleep(100);
                } catch (Exception e) {
                }
            }*/

            do {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be - on right turn.
                //   Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
            } while (!pidRotate.onTarget());
        } else    // left turn.
            do {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be + on left turn.
                //  Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(-power);
                Motor_FR.setPower(-power);
                Motor_BL.setPower(-power);
                Motor_BR.setPower(-power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);

        //rotation = getAngle();

        // wait for rotation to stop.
        try {
            sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // reset angle tracking on new heading.
        resetAngle();
        Log.i(TAG, "Exit Function: rotate");
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateRight(int degrees, double power) {
        Log.i(TAG, "Enter Function: rotate, Angle: " + degrees);

        double angle;
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       /* Motor_FL.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_BL.setDirection(DcMotorEx.Direction.REVERSE);
        Motor_BR.setDirection(DcMotorEx.Direction.REVERSE);*/
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
          /*  while (getAngle() == 0)
            {
                // set power to rotate.
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
                Log.i(TAG, "Function: rotate, Angle less then 0 Motor Power set to: " + power);
                try {
                    sleep(100);
                } catch (Exception e) {
                }
            }*/

            do {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be - on right turn.
                //   Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(-power);
                Motor_FR.setPower(-power);
                Motor_BL.setPower(-power);
                Motor_BR.setPower(-power);
            } while (!pidRotate.onTarget());
        } else    // left turn.
            do {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be + on left turn.
                //  Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);

        //rotation = getAngle();

        // wait for rotation to stop.
        try {
            sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // reset angle tracking on new heading.
        resetAngle();
        Log.i(TAG, "Exit Function: rotate");
    }

    public void teleOpMotorBehavior() {
        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void startIntake(int time) {

        //Set power of all motors
        intake.setPower(0.6);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        intake.setPower(0);
        Log.i(TAG, "Exit Function: moveBackwardForTime");
    }

    public void motorTest(){
        Motor_FL.setPower(1);
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FL.setPower(0);

        Motor_FR.setPower(1);
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_FR.setPower(0);

        Motor_BL.setPower(1);
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_BL.setPower(0);

        Motor_BR.setPower(1);
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Motor_BR.setPower(0);
    }
    public void extendintake(int time) {
        inslide.setPower(-0.75);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inslide.setPower(0);
    }

    public void retractintake(int time) {
        inslide.setPower(0.75);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inslide.setPower(0);
    }

    public void extend(){
        inslide.setPower(-1);
        try {
            sleep(200);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inflip.setPower(1);
        try {
            sleep(900);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inflip.setPower(0);
        inslide.setPower(0);
    }

    public void home(){
        inflip.setPower(-1);
        try {
            sleep(200);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inslide.setPower(1);
        try {
            sleep(800);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inflip.setPower(0);
        try {
            sleep(100);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inslide.setPower(0);
    }

    public void drop(){
        outslide.setPower(-1);
        try {
            sleep(1100);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
        outflip.setPower(0.4);
        try {
            sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0);
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(-0.4);
        try {
            sleep(400);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0);
        outslide.setPower(1);
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
    }
    public void dropMiddle(){
        outslide.setPower(0);
        outflip.setPower(0);

        outslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outslide.setTargetPosition(-675);
        outflip.setTargetPosition(-300);

        outslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outslide.setPower(0.7);
        outflip.setPower(0.1);

        try {
            sleep(1500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        outslide.setTargetPosition(0);
        outflip.setTargetPosition(0);

    }
    public void dropBottom(){
        outflip.setPower(0);

        outflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outflip.setTargetPosition(-300);

        outflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outflip.setPower(0.2);

        try {
            sleep(1500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        outflip.setTargetPosition(0);
    }
    public void perfectDrop(){
        Log.i(TAG, "Outslide initial: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip initial: " + outflip.getCurrentPosition());

        // stops from slipping
        outslide.setPower(-0.02);
        // flip the block about 30 degrees
        outflip.setPower(0.4);
        try {
            sleep(250);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0);
        // Raise up the slide to max height
        outslide.setPower(-1);
        Log.i(TAG, "Outslide 2: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 2: " + outflip.getCurrentPosition());
        try {
            sleep(1100);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
        // Flip completely into the shipping hub
        outflip.setPower(0.4);
        Log.i(TAG, "Outslide 3: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 3: " + outflip.getCurrentPosition());
        try {
            sleep(250);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0);
        Log.i(TAG, "Outslide 4: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 4: " + outflip.getCurrentPosition());
        // Wait one second for block to fall out
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        // Flip backwards back to initial position
        outflip.setPower(-0.4);
        Log.i(TAG, "Outslide 5: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 5: " + outflip.getCurrentPosition());
        try {
            sleep(350);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0);
        //Bring slide back down
        outslide.setPower(1);
        Log.i(TAG, "Outslide 6: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 6: " + outflip.getCurrentPosition());
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
        Log.i(TAG, "Outslide 7: " + outslide.getCurrentPosition());
        Log.i(TAG, "Outflip 7: " + outflip.getCurrentPosition());
    }

    public void perfectDropEncoder() {
        outslide.setPower(0);
        outflip.setPower(0);

        outslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outslide.setTargetPosition(-1350);
        outflip.setTargetPosition(-300);

        outslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outslide.setPower(0.7);
        outflip.setPower(0.2);

        try {
            sleep(1500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        outslide.setTargetPosition(0);
        outflip.setTargetPosition(0);

//        while((outslide.isBusy() || outflip.isBusy())) { }
//
//        outslide.setPower(0);
//        outflip.setPower(0);
    }

    public void perfectIntakeOut() {

        inslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inslide.setPower(0);
        inflip.setPower(0);

        inslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        inslide.setTargetPosition(-1600);
        inflip.setTargetPosition(3500);

        inslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inflip.setPower(1);
        inslide.setPower(0.6);

        //inslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //inslide.setTargetPosition(0);
        //inflip.setTargetPosition(0);
    }
    public void perfectIntakeIn() {

        inslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inslide.setPower(0);
        inflip.setPower(0);

        inslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inflip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        inslide.setTargetPosition(1600);
        inflip.setTargetPosition(-3500);

        inslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inflip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inflip.setPower(1);
        inslide.setPower(0.4);

        //inslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //inslide.setTargetPosition(0);
        //inflip.setTargetPosition(0);
    }
    public void setInflipPower(double power) {
        inflip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inflip.setPower(power);
    }

    public void logOuttakeEncoders() {
        Log.i(TAG, "outslide: " + outslide.getCurrentPosition());
        Log.i(TAG, "outflip: " + outflip.getCurrentPosition());
    }

    public void logIntakeEncoders() {
        Log.i(TAG, "inslide: " + inslide.getCurrentPosition());
        Log.i(TAG, "inflip: " + inflip.getCurrentPosition());
    }

    public void resetOuttakeEncoders() {
        outslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetIntakeEncoders() {
        inslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inflip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void vumarkDetection(VuforiaTrackables targetsFreightFrenzy, double MM_PER_INCH) {
        // Look for first visible target, and save its pose.
        targetFound = false;
        for (VuforiaTrackable trackable : targetsFreightFrenzy)
        {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
            {
                targetPose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getVuforiaCameraFromTarget();

                // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                if (targetPose != null)
                {
                    targetFound = true;
                    targetName  = trackable.getName();
                    VectorF trans = targetPose.getTranslation();

                    // Extract the X & Y components of the offset of the target relative to the robot
                    targetX = trans.get(0) / MM_PER_INCH; // Image X axis
                    targetY = trans.get(2) / MM_PER_INCH; // Image Z axis

                    // target range is based on distance from robot position to origin (right triangle).
                    targetRange = Math.hypot(targetX, targetY);

                    // target bearing is based on angle formed between the X axis and the target range line
                    targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                    break;  // jump out of target tracking loop if we find a target.
                }
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", " %s", targetName);
            telemetry.addData("Range",  "%5.1f inches", targetRange);
            telemetry.addData("Bearing","%3.0f degrees", targetBearing);
            telemetry.addData("X",  "%5.1f inches", targetX);
            telemetry.addData("Y",  "%5.1f inches", targetY);
        } else {
            telemetry.addData(">","Drive using joystick to find target\n");
        }
//
////         Drive to target Automatically if Left Bumper is being pressed, AND we have found a target.
//        if (gamepad1.left_bumper && targetFound) {
//
//            // Determine heading and range error so we can use them to control the robot automatically.
//            double  rangeError   = (targetRange - DESIRED_DISTANCE);
//            double  headingError = targetBearing;
//
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive = rangeError * SPEED_GAIN;
//            turn  = headingError * TURN_GAIN ;
//
//            telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
//        } else {
//
//            // drive using manual POV Joystick mode.
//            drive = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//            turn  =  gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
//            telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
//        }
//        telemetry.update();
//
//        // Calculate left and right wheel powers and send to them to the motors.
//        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        double rightPower   = Range.clip(drive - turn, -1.0, 1.0);
//
//        sleep(10);
    }
    public void raiseintake(int time) {
        inflip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inflip.setPower(-1);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inflip.setPower(0);
    }

    public void lowerintake(int time) {
        inflip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inflip.setPower(1);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        inflip.setPower(0);
    }

    public void raiseouttake(int time) {
        outslide.setPower(-1);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
    }

    public void lowerouttake(int time) {
        outslide.setPower(1);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outslide.setPower(-0.02);
    }

    public void dropdown(int time) {
        outflip.setPower(0.5);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0.02);
    }

    public void dropup(int time) {
        outflip.setPower(-0.5);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        outflip.setPower(0.02);
    }

    public void carouselmove(int time) {
        carousel.setPower(-1);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        carousel.setPower(0);
    }
}