package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
@Disabled
public class rightAutonomous extends LinearOpMode {

    public String TAG = "FTC";
    //---------------------------------------------------------------------------------------

    public void runOpMode() {

        Robot r = new Robot(hardwareMap, telemetry);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();


//BEGINNING AUTONOMOUS CODE
        /*
        Our autonomous code for the Building Side Blue is quick and simple
            1) First, we move left to get our clamping mehcanism right up next to the foundation
            2) We then clamp down and move back to the wall
                - From here we right align with the side wall
                - We then front align with the back wall to be sure of our relative position
            3) To make sure that the foundation is in the area, we go in front of the foundation and
               drive backwards. This makes sure that the wall is right up with the wall
            4) To end, we go park on the middle line
            4) In total, our team can score 15 points from this Loading zone program
         */

        //Vuforia detection
        //int detected_rings = 1;

        r.moveF(1, 1000);
        r.moveL(1, 100);
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


    }}
