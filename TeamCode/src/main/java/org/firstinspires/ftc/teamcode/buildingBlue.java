package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.robotcore.external.ClassFactory;


@Autonomous
@Disabled
public class buildingBlue extends LinearOpMode {

    public String TAG = "FTC";
    //---------------------------------------------------------------------------------------

    public void runOpMode() {

        Robot Robot = new Robot(hardwareMap, telemetry);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        Log.i(TAG, "*************Starting Autonomous TEST**************************");


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

Robot.moveBackwardForTime(1,500,true);
sleep(100);
     /* Robot.moveLeftForTime(0.5, 750, false);
      Robot.moveForwardToPosition(0.5, 32);
      //Robot.ClampDown(250);
      Robot.moveBackwardToPosition(0.6, 30);
      Robot.slowTurn(240);
      //Robot.ClampUp(250);
      //Robot.moveRightForTime(1, 500, false);
      //Robot.moveBackwardForTime(0.3, 500, false);
      //Robot.moveRightForTime(0.5, 3000, false);
    }*/}}