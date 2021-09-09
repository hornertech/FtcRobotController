package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
public class loadingRed extends LinearOpMode {

    public String TAG = "FTC";
    private static final String VUFORIA_KEY = "AV8zEej/////AAABmVo2vNWmMUkDlkTs5x1GOThRP0cGar67mBpbcCIIz/YtoOvVynNRmJv/0f9Jhr9zYd+f6FtI0tYHqag2teC5GXiKrNM/Jl7FNyNGCvO9zVIrblYF7genK1FVH3X6/kQUrs0vnzd89M0uSAljx0mAcgMEEUiNOUHh2Fd7IOgjlnh9FiB+cJ8bu/3WeKDxnDdqx6JI5BlQ4w7YW+3X2icSRDRlvE4hhuW1VM1BTPQgds7OtHKqUn4Z5w1Wqg/dWiOHxYTww28PVeg3ae4c2l8FUtE65jr2qQdQNc+DMLDgnJ0fUi9Ww28OK/aNrQQnHU97TnUgjLgCTlV7RXpfut5mZWXbWvO6wA6GGkm3fAIQ2IPL";
    private VuforiaLocalizer vuforia = null;

    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    //Define Measurements
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private double location[] = {0, 0, 0, 0, 0, 0, 0};
    // Location[0] : 0 = Target not visible, 1 = Target visible
    // Location[1] : Lateral drift from target
    // Location[2] : Distance from Target
    // Location[3] : Vertical shift, not needed for our program
    // Location[4] : Vertical Angle Shift; Not useful for our program
    // Location[5] : Upward Angle Shift; Not useful for our program
    // Location[6] : Horizontal Angle Shift
    private int boardDistance = 30;
    private int bridgeOffset = 10;
    private int skystonePicked = 0;
    private int skystoneLocation = 0;
    private int stoneStrafeTime = 650;
    private int stoneForwardTime = 175;
    private double correctionDistance = 0;
    private int secondSkyStoneLocation = -1;
    // Vuforia Code
    /* Vuforia is a detection program used to detect the skystones by our team. We find it very useful as
       it can tell us if a skystone is in fron of our camera, as well as the various values mentioned above.
       These values, such as the Horizontal Angle Shift, Lateral Shift and Distance from Target allow us to
       correct our robot to perfectly pick up the skystones as well as efficiently deliver them to the building
       zone.
    */

    void detectOnce(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible. -- Our only Trackable is Skystone
        Log.i(TAG, "Entering Function detectOnce");
        targetVisible = false;
        location[0] = 0;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            location[0] = 1;
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            location[1] = translation.get(0) / mmPerInch;
            location[2] = translation.get(1) / mmPerInch;
            location[3] = translation.get(2) / mmPerInch;
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            location[4] = rotation.firstAngle;
            location[5] = rotation.secondAngle;
            location[6] = rotation.thirdAngle;
            Log.i(TAG, "Positions: X =" + location[1] + " Y = " + location[2] + " Z = " + location[3]);
            Log.i(TAG, "Angles: ROLL =" + location[4] + "Pitch = " + location[5] + "Heading = " + location[6]);
            if ((location[1] > 35 || location[1] < -35) || (location[2] > 10 || location[2] < -10) || (location[6] >30 || location[6] < -30))
            {
                location[0] = 0;
                Log.i(TAG, "Exiting Function detectOnce: Detected wrong Element, returning no element detected");
            }
            else {
                Log.i(TAG, "Exiting Function detectOnce: Returning element detected");
            }
        } else {
            telemetry.addData("Visible Target", "none");
            Log.i(TAG, "No Target Visible");
        }
        telemetry.update();
    }
    /* Move to skystone is our correction program;
       This program uses the angle shift, lateral shift, and distance from target recorded
       by Vuforia to move the robot to the correct position.
    */

    public void moveToSkyStone(Robot robot) {

        Log.i(TAG, "Entering Function moveToSkyStone");
        //Correct Lateral Shift
        if (location[2] > 0)
        {
            correctionDistance = (location[2]);
            robot.moveRightToPosition(0.5, Math.abs(correctionDistance));
            //robot.moveRightForTime(0.5, (int)(correctionDistance*80), false);

            skystoneLocation--;

        }
        else if (location[2] < 0)
        {
            correctionDistance = (Math.abs(location[2]));
            robot.moveLeftToPosition(0.5, Math.abs(correctionDistance));
            //robot.moveLeftForTime(0.5, (int)(correctionDistance*80), false);
            if (location[2] > -3 ){
                skystoneLocation--;
            }
            else if (location[2] < -12 ){
                skystoneLocation++;
            }
        }

        sleep(450);
        robot.fixOrientation(0);

        //Move Forward to Target
        robot.moveForwardToPosition(0.75, (Math.abs (location[1]) - 1));

        location[0]  = 0;
        Log.i(TAG, "Exiting Function moveToSkyStone");
    }


    //The Autonomous Program
    public void runOpMode() {
        int i;

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Load data sets for trackable objects; data sets found in 'assets'
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        //Load stone target object
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // Set Stone Target Position
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 7.5f * mmPerInch;     // eg: Camera is ON the robot's center line

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // We need to rotate the camera around its long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //confirm position of phone
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        Robot Robot = new Robot(hardwareMap, telemetry);


        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        Log.i(TAG, "*************Starting Autonomous TEST**************************");

        targetsSkyStone.activate();

//BEGINNING AUTONOMOUS CODE
        /*
        Our autonomous code for the Loading Side Blue has multiple facets
            1) First, we begin by moving forward and raising our slides, getting into a position
            to detect
            2) We then begin our detection For Loop
                - This loop goes 5 times, looking for a skystone
                - If the skystone is found, we run moveToSkystone to get into a position to grab
                - Next, we cross the bridge, attempt to drop the skystone on top of the foundation
                  if it has been placed in the triangle area, then return back and get ready for the
                  next detection
            3) After we drop the second stone, we park on the middle line
            4) In total, our team can score 33 points from this Loading zone program
         */

        //Robot.moveWithSlide(0.4, 650, 1, 1, 1);
        //Robot.moveForwardToPosition(1, 12);
        //Robot.moveForwardForTime(0.5, 650, false );
        // Robot.moveSlides(1,700, false);


        // Robot.dropStone();
        //Our Detection Algorithm
        for (i = skystoneLocation; i < 6; i++) {

            if(secondSkyStoneLocation != 5){
                sleep(400);
                detectOnce(allTrackables);
                detectOnce(allTrackables);
            }
            //Detect Skystone
            //Check if skystone has been detected
            if (location[0] == 1) {
                skystoneLocation = i;
                skystonePicked++;

                //Robot.moveSlides(-1, 700, false);
                moveToSkyStone(Robot); // Runs correction program to get to the skystone
                if (skystonePicked == 1) {
                    secondSkyStoneLocation = skystoneLocation + 3;
                }
                Log.i(TAG, "++++++Detected Stone at Location : " + (skystoneLocation + 1) + " index: " + i + "Second Skystone: " + (secondSkyStoneLocation + 1));

                Robot.moveBackwardForTime(1, 120, false );
                Robot.rotateRight(-91, 1);

                if(skystonePicked == 1) {
                    //Robot.moveForwardForTime(1, 600 + ((  * stoneForwardTime), false);
                    Robot.moveForwardToPosition(1, 38 + 8*skystoneLocation);
                }
                else{
                    //Robot.moveForwardForTime(1, 600 + (secondSkyStoneLocation + 1) * stoneForwardTime, false);
                    Robot.moveForwardToPosition(1, 38 + 8*secondSkyStoneLocation);
                }

                if (skystonePicked == 2) {
                    // Delivered both skystones, go park
                    Robot.moveBackwardForTime(1, 180, false);
                    skystoneLocation = 6;
                    break;
                } else {
                    // First skystone delivered, go back to find the second one
                    Robot.moveBackwardToPosition(1, 38 + secondSkyStoneLocation*8);
                    if (secondSkyStoneLocation == 5){
                        Robot.moveForwardForTime(0.5, 100, false);
                    }
                    // Robot.moveSlides(1, 700, false);
                    Robot.rotateLeft(85, 1);
                    Robot.moveBackwardForTime(0.5, 100, false );
                    i = secondSkyStoneLocation-1;
                }
                // In the case that Skystone was not detected
            } else {
                // Stone in front is not skystone, move to next one
                 Robot.slowTurn(-5.0);
                //Robot.moveRightForTime(0.5, stoneStrafeTime, false);
                Robot.moveLeftToPosition(0.5, 8);
            }

            if (i == 5 & skystonePicked != 2) {
                // If you haven't detected 2 stones, try and get 6th stone
                Log.i(TAG, "+++++++ Second Stone not detected, picking from: " + (secondSkyStoneLocation+1));
                if(secondSkyStoneLocation == -1 || secondSkyStoneLocation == 4 || secondSkyStoneLocation > 5){

                    sleep(500);
                    Robot.fixOrientation(0);
                    detectOnce(allTrackables);
                    if (location[0] == 1)
                    {
                        moveToSkyStone(Robot);
                    }
                    else {
                        Robot.moveForwardToPosition(0.5, 9);
                    }


                    Robot.moveBackwardForTime(1, 250, false);
                }
                else if(secondSkyStoneLocation == 3){
                    //Robot.moveLeftForTime(0.5, 840, false);

                    sleep(500);
                    Robot.fixOrientation(0);
                    detectOnce(allTrackables);
                    if (location[0] == 1)
                    {
                        moveToSkyStone(Robot);
                    }
                    else {
                        Robot.moveForwardToPosition(0.5, 9);
                    }


                    sleep(200);
                    Robot.moveBackwardForTime(1, 250, false);
                }
                else if(secondSkyStoneLocation == 5) {
                    sleep(200);
                    detectOnce(allTrackables);

                    //Robot.moveForwardForTime(0.3, 200, false);
                    if(location[0] == 0) {
                        Robot.moveForwardToPosition(0.6, 8);
                    }
                    if(location[0] == 1) {
                        Robot.moveForwardToPosition(0.6, (Math.abs (location[1])));
                    }
                    Robot.slowTurn(25);

                    sleep(200);
                    Robot.moveBackwardForTime(1, 250, false);
                }
            }
        }
        targetsSkyStone.deactivate();
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

}