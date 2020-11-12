package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot extends java.lang.Thread {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //private static final int TICKS_PER_ROTATION = 1440; //Tetrix motor specific
    private static final int TICKS_PER_ROTATION = 1120; //Tetrix motor specific
    private static final double WHEEL_DIAMETER = 4; //Wheel diameter in inches
    private String TAG = "FTC";

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;
    private DcMotorEx shooter;
    private DcMotorEx wobbler; 

    private DcMotor Slide_L;
    private DcMotor Slide_R;

    //private DcMotor Clamp_L;
    //private DcMotor Clamp_R;

    private Servo pincher;
    private Servo capper;
    // The IMU sensor object
    private BNO055IMU imu;

    private final int tollerance = 15;

    // State used for updating telemetry
    private Orientation     angles;
    private PIDController   pidRotate, pidDrive;
    private Orientation     lastAngles = new Orientation();
    private double          globalAngle, correction;

    public boolean isTeleOp     = true;
    private boolean DEBUG_DEBUG  = false;
    private boolean DEBUG_INFO   = false;

    private long     movementFactor      = 1;
    private double   turnFactor          = 7.2;
    private double   leftStrafeFactor    = 1.2;
    private double   rightStrafeFactor   = 1.2;

    Robot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDevices();
    }

    /* Calculate Drivetrain PID cofficients */

    private final int motorFLMaxSpeed = 3320;
    double motorFLF = 32767/ (double)motorFLMaxSpeed;
    double motorFLP = 0.1 * motorFLF;
    double mototFLI = 0.1 * motorFLP;

    private final int motorFRMaxSpeed = 3360;
    double motorFRF = 32767/ (double)motorFRMaxSpeed;
    double motorFRP = 0.1 * motorFRF;
    double mototFRI = 0.1 * motorFRP;

    private final int motorBLMaxSpeed = 3360;
    double motorBLF = 32767/ (double)motorBLMaxSpeed;
    double motorBLP = 0.1 * motorBLF;
    double mototBLI = 0.1 * motorBLP;

    private final int motorBRMaxSpeed = 3320;
    double motorBRF = 32767/ (double)motorBRMaxSpeed;
    double motorBRP = 0.1 * motorBRF;
    double mototBRI = 0.1 * motorBRP;


    private void initDeviceCore() throws Exception {

        telemetry.addData("Please wait", "In function init devices");
        telemetry.update();

        //Wheels
        Shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Wobbler = hardwareMap.get(DcMotorEx.class, "wobbler");
        Motor_FL = hardwareMap.get(DcMotorEx.class, "motor_fl");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "motor_fr");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "motor_br");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "motor_bl");

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

        Slide_R = hardwareMap.get(DcMotor.class, "slide_r");
        Slide_L = hardwareMap.get(DcMotor.class, "slide_l");

        //Clamp_L = hardwareMap.get(DcMotor.class, "clamp_l");
        //Clamp_R = hardwareMap.get(DcMotor.class, "clamp_r");

        pincher = hardwareMap.get(Servo.class, "pincher");
        capper = hardwareMap.get(Servo.class, "capper");
      /*  Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        BNO055IMU.Parameters parametersIMU  = new BNO055IMU.Parameters();
        parametersIMU.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.calibrationDataFile   = "BNO055IMUCalibration.json";
        parametersIMU.loggingEnabled        = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.0099, .0001, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);
        

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Start Orientation First : "+ angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle );

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

        int encoder_ticks = (int) ((java.lang.Math.abs(angle) * TICKS_PER_ROTATION ) / 360);

        Log.i(TAG, "Ticks needed for Angle : " + encoder_ticks);
        Log.i(TAG, "Exit FUNC: AngleToTick");

        return (encoder_ticks);
    }

    boolean drivetrainBusy(int ticks)
    {
        int avg = (java.lang.Math.abs(Motor_FL.getCurrentPosition())
                + java.lang.Math.abs(Motor_FR.getCurrentPosition())
                + java.lang.Math.abs(Motor_BL.getCurrentPosition())
                + java.lang.Math.abs(Motor_BR.getCurrentPosition()))/4;
        if ((avg >= (ticks - tollerance))  || (avg <= (ticks + tollerance))){
            return false;
        }
        return true;
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
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000){
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
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for them to reach to the position
        // while ((Motor_FL.isBusy() && Motor_BL.isBusy()) || (Motor_FR.isBusy() && Motor_BR.isBusy())){
        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {

            if ((System.currentTimeMillis() - startTime) > 3000){
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
    public void moveRightToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveRightToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();
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
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //Wait for them to reach to the position
        // while ((Motor_FR.isBusy() && Motor_BL.isBusy()) || (Motor_FL.isBusy() && Motor_BR.isBusy())){
        while ((Motor_FL.isBusy()
                && Motor_FR.isBusy()
                && Motor_BL.isBusy()
                && Motor_BR.isBusy())
                &&((System.currentTimeMillis() - startTime) < 1000)) {

            if ((System.currentTimeMillis() - startTime) > 1000){
                Log.i(TAG, "moveRightToPosition : breaking out as time is more than 3 sec");
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

    // Move Right to specific distance in inches, with power (0 to 1)
    public void moveLeftToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        long startTime = System.currentTimeMillis();
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
        Motor_BL.setPower(power);

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
                && Motor_BR.isBusy())
                &&((System.currentTimeMillis() - startTime) < 1000)) {


            if ((System.currentTimeMillis() - startTime) > 1000){
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

    public void moveLeftForTime(double power, int time, boolean speed) {
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

    public void moveRightForTime(double power, int time, boolean speed) {
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

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set power of all motors
        Motor_FL.setPower(orientation * power);
        Motor_FR.setPower(orientation * power);
        Motor_BR.setPower(orientation * power);
        Motor_BL.setPower(orientation * power);

       /* try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);*/
    }

    public void turnOff(){
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }
    public void turnOffSlides(){
        Slide_L.setPower(0);
        Slide_R.setPower(0);
    }

    public void moveF(double power, long distance) {
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

    public void moveL(double power, long distance) {

        Motor_FL.setPower((1) * power );
        Motor_FR.setPower((1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1) *  power);
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

    public void moveR(double power, long distance) {

        Motor_FL.setPower((-1) * (1) * power);
        Motor_FR.setPower((-1) * (1) * power);
        Motor_BR.setPower(power );
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
        telemetry.addData("Direction", "Left");
        telemetry.update();
        if (isTeleOp == false) pause(250);*/
    }

    /*public void ClampDown(int time){
        Clamp_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Clamp_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Clamp_L.setPower(0.4);
        Clamp_R.setPower(-0.4);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Clamp_L.setPower(0);
        Clamp_R.setPower(0);

    }
    public void ClampUp(int time){
        Clamp_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Clamp_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Clamp_L.setPower(-0.4);
        Clamp_R.setPower(0.4);
        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }
        Clamp_L.setPower(0);
        Clamp_R.setPower(0);
    }*/

    public void slowTurn(double angle) {
        Log.i(TAG, "Enter Function slowTurn Angle: "+ angle);
        int sleepTime = java.lang.Math.abs ((int)(angle * turnFactor));

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
                Log.i(TAG, "SlowTurn Sleep: "+ sleepTime);
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
                Log.i(TAG, "SlowTurn Sleep: "+ sleepTime);
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

    public void grabStone() {
        Log.i(TAG, "Enter Function grabStone Start Position:" + pincher.getPosition());
        pincher.setPosition(0.4);
        Log.i(TAG, "Exit Function grabStone End Position:" + pincher.getPosition());
    }

    public void tolerance(){
            int tolerance1;
            tolerance1 = Motor_FL.getTargetPositionTolerance();
            Log.i(TAG, "" + tolerance1);

    }
    public void dropStone() {
        Log.i(TAG, "Enter Function dropStone Start Position:" + pincher.getPosition());
        pincher.setPosition(1);
        Log.i(TAG, "Exit Function dropStone Start Position:" + pincher.getPosition());
    }
    public void dropCap(){
        capper.setPosition(0.4);
    }
    public void closeCap(){ capper.setPosition(0.8); }

     public void moveWithSlide(double power, int time,int direction, double slidePower, int slideDirection) {
        Log.i(TAG, "Enter Function: moveWithSlide1 ");
        // Reset all encoders
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Slide_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_FL.setPower((direction) *  power);
        Motor_FR.setPower((direction) * (-1) *power);
        Motor_BR.setPower((direction) * (-1) *power);
        Motor_BL.setPower((direction) *  power);

        Slide_L.setPower((slideDirection) * slidePower);
        Slide_R.setPower((slideDirection) * (-1) * slidePower);

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
        Slide_R.setPower(0);
        Slide_L.setPower(0);
    }

    public void fixOrientation(double degree)
    {

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Current Orientation First : "+ angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle );
        double diff = degree - angles.firstAngle;
        if (diff < 10 || diff > -10) {
            slowTurn(diff);
        }

    }

    

    public void moveSlides(double power, int time, boolean teleop) {
        Log.i(TAG, "Enter Function: moveSlides");
        // Reset all encoders
        long slide_R_Start = Slide_R.getCurrentPosition();
        long slide_L_Start = Slide_L.getCurrentPosition();

        Slide_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (teleop) {
            Slide_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Slide_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            Slide_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slide_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Slide_L.setPower(power);
        Slide_R.setPower((-1) * power);

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Slide_L.setPower(0);
        Slide_R.setPower(0);

        Log.i(TAG, "Exit Function: moveSlides");
    }
    public void moveSlidesTeleOp(double power, int time, boolean teleop) {
        Log.i(TAG, "Enter Function: moveSlides");
        // Reset all encoders
        long slide_R_Start = Slide_R.getCurrentPosition();
        long slide_L_Start = Slide_L.getCurrentPosition();

        Slide_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (teleop) {
            Slide_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Slide_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            Slide_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slide_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        Slide_L.setPower(power);
        Slide_R.setPower((-1) * power);

        Log.i(TAG, "Exit Function: moveSlides");
    }

    /* IMU based turn functions */


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
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
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateLeft(int degrees, double power)
    {
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

        if (degrees < 0)
        {
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

            do
            {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be - on right turn.
                //   Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
            } while (!pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
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
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateRight(int degrees, double power)
    {
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

        if (degrees < 0)
        {
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

            do
            {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be - on right turn.
                //   Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(-power);
                Motor_FR.setPower(-power);
                Motor_BL.setPower(-power);
                Motor_BR.setPower(-power);
            } while (!pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
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
}

