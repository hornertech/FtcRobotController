package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;



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
public class vuforiaTest extends LinearOpMode {

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

    private int boardDistance = 30;
    private int bridgeOffset = 10;
    private int skystonePicked = 0;
    private int skystoneLocation = 0;
    private int stoneStrafeTime = 650;
    private int stoneForwardTime = 255;
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
//
//            location[4] = rotation.firstAngle;
//            location[5] = rotation.secondAngle;
//            location[6] = rotation.thirdAngle;
            Log.i(TAG, "Positions: X =" + location[1] + " Y = " + location[2] + " Z = " + location[3]);
//            Log.i(TAG, "Angles: ROLL =" + location[4] + "Pitch = " + location[5] + "Heading = " + location[6]);
//            if ((location[1] > 20 || location[1] < -20) || (location[2] > 10 || location[2] < -10) || (location[6] >30 || location[6] < -30))
//            {
//                location[0] = 0;
//                Log.i(TAG, "Exiting Function detectOnce: Detected wrong Element, returning no element detected");
//            }
//            else {
//                Log.i(TAG, "Exiting Function detectOnce: Returning element detected");
//            }
        } else {
            telemetry.addData("Visible Target", "none");
            Log.i(TAG, "No Target Visible");
        }
        telemetry.update();
    }


    //The Autonomous Program
    public void runOpMode() {
        int i;

        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Load data sets for trackable objects; data sets found in 'assets'
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        //Load stone target object
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // Set Stone Target Position
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 7.5f * mmPerInch;     // eg: Camera is ON the robot's center line

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // We need to rotate the camera around its long axis to bring the correct camera forward.
//        if (CAMERA_CHOICE == BACK) {
//            phoneYRotate = -90;
//        } else {
//            phoneYRotate = 90;
//        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
//        if (PHONE_IS_PORTRAIT) {
//            phoneXRotate = 90;
//        }
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
        while (true) {
            sleep(1000);
            detectOnce(allTrackables);
        }

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

