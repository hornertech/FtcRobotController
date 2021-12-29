package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class FullAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot Robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        int DETECT = 1;
        Robot.moveBackwardToPosition(0.5, 46);
        Robot.arcTurn(-0.5,1250);
        Robot.moveForwardToPosition(0.3, 13);
        //Robot.moveForwardForTime(0.1, 1000, false);
        if(DETECT == -1){
            Robot.dropBottom();
        }
        else if (DETECT == 0){
            Robot.moveForwardToPosition(0.3, 4);
            Robot.dropMiddle();
            Robot.moveBackwardToPosition(0.3, 4);
        }
        else{
            Robot.moveForwardToPosition(0.3, 6);
            Robot.drop();
            Robot.moveBackwardToPosition(0.3, 6);
        }
        Robot.arcTurn(-0.5,1400);
        Robot.moveLeftToPosition(0.7, 34);
        Robot.moveBackwardToPosition(0.8,10);
        Robot.moveLeftForTime(0.3, 1000, false);
        //Robot.moveRightForTime(0.5, 250, false);

        if(DETECT == -1){
            Robot.moveBackwardToPosition(0.3, 24);
            Robot.moveBackwardForTime(0.1, 1000, false);
        }
        else{
            Robot.moveBackwardToPosition(0.4, 23);
            Robot.moveBackwardForTime(0.1, 300, false);
        }

        Robot.carouselmove(3600);
        Robot.moveForwardToPosition(0.5, 6);
        Robot.arcTurn(-0.5,1250);
        Robot.moveBackwardToPosition(0.8, 14);
        Robot.moveLeftForTime(0.4, 2800, false);
        Robot.moveBackwardToPosition(1, 32);
        Robot.moveLeftForTime(0.2, 600, false);
        Robot.moveBackwardForTime(0.5, 2600, false);
    }
}
