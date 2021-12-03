package org.firstinspires.ftc.teamcode;
//Fix if detecting 2 or 0 minerals
//Give Power to Servo Motor holder
//Buttons to move latch and slide
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleopTest extends LinearOpMode{

    @Override
    public void runOpMode()  {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot (hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData ("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();
        //Driver must press INIT and then ▶️

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();

            /********** GamePad 1 ****************/
            //Turning
            if(this.gamepad1.left_stick_x < 0.5 && this.gamepad1.left_stick_x > -0.5){
                robot.turnOff();
            }
            if (this.gamepad1.left_stick_x > 0.5) {
                robot.turnForTime(0.8, 10, false, -1 );
            }

            if (this.gamepad1.left_stick_x < -0.5) {
                robot.turnForTime(0.8, 10, false, 1 );
            }

            // Moving
            if(this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5){
                robot.turnOff();
            }
            if (this.gamepad1.right_stick_y > 0.5) {
                robot.moveF(1, 10);
            }


            if (this.gamepad1.right_stick_y < -0.5) {
                robot.moveB(1, 10);
            }

            if (this.gamepad1.right_stick_x > 0.5) {
                robot.moveR(1, 10);
            }

            if (this.gamepad1.right_stick_x < -0.5) {
                robot.moveL(1, 10);
            }

            if (this.gamepad1.left_trigger > 0.5) {
                robot.moveForwardToPosition(0.4, 24);
                sleep(2000);
                robot.moveBackwardToPosition(0.4, 24);
            }
            if (this.gamepad1.right_trigger > 0.5) {
                robot.moveForwardForTime(0.4, 1000, false);
                sleep(2000);
                robot.moveBackwardForTime(0.4, 1000, false);
            }
            /*
            if (this.gamepad1.b == true) {
                robot.startShoot();
            }
            if (this.gamepad1.b == false){
                robot.endShoot();
            }*/

            if (this.gamepad2.b == true){
                robot.lowerintake(50);
            }
            if (this.gamepad2.a == true){
                robot.raiseintake(50);
            }
            if (this.gamepad2.x == true){
                robot.extend();
            }
            if (this.gamepad2.y == true){
                robot.home();
            }
            if (this.gamepad2.dpad_up){
                robot.startIntake(100);
            }
            if (this.gamepad2.dpad_down){
                robot.carouselmove(20000);
            }
            if(this.gamepad2.dpad_left == true) {
                robot.retractintake( 100);
            }
            if(this.gamepad2.dpad_right == true) {
                robot.extendintake(100);
            }

            if(this.gamepad1.dpad_up == true) {
                robot.moveF( 0.5,10);
            }

            if(this.gamepad1.dpad_down == true) {
                robot.moveB(0.5, 10);
            }
            if(this.gamepad1.a == true){
                robot.startIntake(100);
            }
            if (this.gamepad1.b == true){
                robot.home();
            }
            if (this.gamepad1.x == true){
                robot.extend();
            }
            if (this.gamepad1.y == true) {
                robot.drop();
            }

            if(this.gamepad2.right_stick_x > 0.5){
            }
            if(this.gamepad2.right_stick_x < -0.5){
            }
            /****************** GamePad 2 **************/
            // Pincher

        };
    };
}