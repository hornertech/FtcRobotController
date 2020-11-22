package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous
@Disabled
public class rightAutonomous extends LinearOpMode {
    public String TAG = "FTC";
    //---------------------------------------------------------------------------------------

    @TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
    @Disabled
    public class ConceptTensorFlowObjectDetection extends LinearOpMode {
        private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
        private static final String LABEL_FIRST_ELEMENT = "Quad";
        private static final String LABEL_SECOND_ELEMENT = "Single";
    
        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code on the next line, between the double quotes.
         */
        private static final String VUFORIA_KEY =
                " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
    
        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        private VuforiaLocalizer vuforia;
    
        /**
         * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
         * Detection engine.
         */
        private TFObjectDetector tfod;
    
        @Override
        public void runOpMode() {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();
            initTfod();
    
            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();
    
                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 1.78 or 16/9).
    
                // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                //tfod.setZoom(2.5, 1.78);
            }
    
            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
            waitForStart();
    
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                          telemetry.addData("# Object Detected", updatedRecognitions.size());
    
                          // step through the list of recognitions and display boundary info.
                          int i = 0;
                          for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                              recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                          }
                          telemetry.update();
                          console.log(telemetry);
                        }
                    }
                }
            }
    
            if (tfod != null) {
                tfod.shutdown();
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
    
            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }
    
        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }
    }

    public void runOpMode() {

        Robot r = new Robot(hardwareMap, telemetry);
        int detected_rings = updatedRecognitions.size();

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


//BEGINNING AUTONOMOUS CODE
//Key:
//{TBW: To be written}

//Inital Movement, static (Not dependent on # of rings)

        r.moveF(1, 900);
        r.moveR(1, 100);

        //If # of rings = 0
        //Shoot x3 (TBW)
        if (detected_rings == 0) {
            r.moveF(1, 100);
            r.moveL(1, 200);
            //Drop wobble
        }

        //If # of rings = 1
        //Shoot x3
        if (detected_rings == 1) {
            r.moveB(1, 400);
            //pickup rings
            r.moveF(1, 400);
            //shoot x1
            r.moveF(1, 600);
            //Drop wobble
            r.moveB(1, 400);
        }

        //If # of rings = 4
        //Shoot x3
        if (detected_rings == 4) {
            r.moveB(1, 400);
            //pickup rings
            r.moveF(1, 400);
            //shoot x3
            r.moveF(1, 600);
            r.moveL(1, 200);
            //Drop wobble
            r.moveB(1, 500);
        }

    }}
