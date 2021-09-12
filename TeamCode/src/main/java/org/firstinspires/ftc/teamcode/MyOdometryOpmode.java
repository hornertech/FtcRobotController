package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

//import org.firstinspires.ftc.teamcode.Robot.Drivetrain.Odometry.OdometryGlobalCoordinatePosition;


/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")


public class MyOdometryOpmode extends LinearOpMode {

    org.firstinspires.ftc.teamcode.Robot Robot = new org.firstinspires.ftc.teamcode.Robot(hardwareMap, telemetry);
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    final double ENCODER_RES = 360;
    final double wheel_diameter = 38/25.4;  //33mm to inches
    final double dis_per_rotation = wheel_diameter * 3.14;
    final double COUNTS_PER_INCH = ENCODER_RES/dis_per_rotation;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "motor_fr", rbName = "motor_br", lfName = "motor_fl", lbName = "motor_bl";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = rbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;


    public void goTOPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError) {
        //PUBLIC or PRIVATE?
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        Log.i("FTC", "robot position X"+ globalPositionUpdate.returnXCoordinate());
        Log.i("FTC", "robot position Y"+ globalPositionUpdate.returnYCoordinate());
        Log.i("FTC", "target Y:"+ targetYPosition);
        telemetry.addData("Trash", targetXPosition);
        telemetry.addData("Trash2", targetYPosition);
        telemetry.update();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        Log.i("FTC", "distance test: "+distance);
        while (opModeIsActive() && distance>allowableDistanceError) {

            right_front.setPower((0.25)); // backwards - towards hub - clockwise
            right_back.setPower((0.25)); // backwards - towards hub - clockwise
            left_front.setPower((-0.25)); // forwards - towards column - clockwise
            left_back.setPower((0.25)); // backwards - towards hub - counterclockwise

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            //Log.i("FTC", "dist to x test: "+distanceToXTarget);

            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            Log.i("FTC", "target Y"+ targetYPosition);
            //Log.i("FTC", "dist to y test: "+distanceToYTarget);

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            //Log.i("FTC", "robot movement x component log: "+robot_movement_x_component);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            //Log.i("FTC", "robot movement y component log: "+robot_movement_y_component);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            //Log.i("FTC", "desired Robot orientation log: "+desiredRobotOrientation);


            //move robot at an ANGLE by distance inches
            //rotate robot to change orientation


            // comments for at power = 1

        }
        right_front.setPower(0); // backwards - towards hub - clockwise
        right_back.setPower(0); // backwards - towards hub - clockwise
        left_front.setPower(0); // forwards - towards column - clockwise
        left_back.setPower(0); // backwards - towards hub - counterclockwise


    }


    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        Log.i("FTC", "starting starting starting");

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();//make sure this works for OUR robot
        globalPositionUpdate.reverseNormalEncoder();

        goTOPosition(0 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0.5, 0,5 * COUNTS_PER_INCH);
        Log.i("FTC", "FINAL robot position X"+ globalPositionUpdate.returnXCoordinate());
        Log.i("FTC", "FINAL robot position Y"+ globalPositionUpdate.returnYCoordinate());

        //goTOPosition(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.5, 0,1 * COUNTS_PER_INCH);?
        //goTOPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.5, 0,1 * COUNTS_PER_INCH);

            sleep(3000);
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.update();

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return speed*Math.sin(Math.toRadians(desiredAngle));
        //return speed*Math.sin(desiredAngle+45);
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return speed*Math.cos(Math.toRadians(desiredAngle));
        //return speed*Math.cos(desiredAngle+45);
    }

    }
