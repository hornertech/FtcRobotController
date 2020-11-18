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
public class movements extends LinearOpMode {
    public String TAG = "FTC";
    //---------------------------------------------------------------------------------------

    @TeleOp(name = "Basic Movements", group = "Concept")
    @Disabled
    public class ConceptTensorFlowObjectDetection extends LinearOpMode {
    
        @Override

    

    public void runOpMode() {

        Robot r = new Robot(hardwareMap, telemetry);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


//BEGINNING AUTONOMOUS CODE
//Key:
//{TBW: To be written}

//Inital Movement, static (Not dependent on # of rings)

        r.moveF(.1, 100);
        r.moveB(.1, 100);
        r.moveR(.1, 100);
        r.moveL(.1, 100);

    }}
