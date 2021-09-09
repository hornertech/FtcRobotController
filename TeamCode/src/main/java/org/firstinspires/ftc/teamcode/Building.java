package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous

public class Building extends LinearOpMode {

    public String TAG = "FTC";
    //---------------------------------------------------------------------------------------

    public void runOpMode() {

        Robot Robot = new Robot(hardwareMap, telemetry);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        Log.i(TAG, "*************Starting Autonomous Building**************************");


        sleep(100);
        Robot.moveBackwardForTime(1, 500, true);
        sleep(100);
    }
}