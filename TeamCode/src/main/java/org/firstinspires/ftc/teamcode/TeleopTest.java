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
            if ((this.gamepad1.left_stick_x > 0.2 && this.gamepad1.left_stick_x != 0)
                    && this.gamepad1.right_stick_x == 0
                    && this.gamepad1.right_stick_y == 0) {
                robot.arcTurn(0.8, 100);
            }
            if ((this.gamepad1.left_stick_x < -0.2 && this.gamepad1.left_stick_x != 0)
                    && this.gamepad1.right_stick_x == 0
                    && this.gamepad1.right_stick_y == 0) {
                robot.arcTurn(-0.8, 100);
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
                robot.moveLeftForTime(0.4, 1000, false);
                sleep(2000);
                robot.moveRightForTime(0.4, 1000, false);
            }
            if (this.gamepad1.right_trigger > 0.5) {
                robot.moveLeftToPosition(0.4, 24);
                sleep(2000);
                robot.moveRightToPosition(0.4, 24);
            }
            if (this.gamepad2.a == true){
                //robot.raiseouttake(1100);
                robot.perfectDrop();
            }
            if (this.gamepad2.b == true){
                robot.raiseouttake(550);
            }
            if (this.gamepad2.x == true){
                robot.lowerouttake(1100);
            }
            if (this.gamepad2.y == true){
                robot.lowerouttake(550);
            }
            if (this.gamepad1.a){
                robot.carouselmove(2200);
            }
            if (this.gamepad1.b){
                robot.carouselmove(1000);
            }
            if(this.gamepad2.dpad_up == true) {
                robot.raiseouttake( 50);
            }
            if(this.gamepad2.dpad_down == true) {
                robot.lowerouttake(50);
            }
            if(this.gamepad2.right_stick_y < -0.2 ){
                robot.extendintake(50);
            }
            if(this.gamepad2.right_stick_y > 0.2 ){
                robot.retractintake(10);
            }
            if(this.gamepad2.left_stick_y < -0.2 ){
                robot.raiseintake(50);
            }
            if(this.gamepad2.left_stick_y > 0.2 ){
                robot.lowerintake(50);
            }
            if(this.gamepad2.left_trigger > 0.5){
                robot.startIntake(100);
            }
            if(this.gamepad2.right_trigger > 0.5){
                robot.dropBottom();
            }
            if(this.gamepad2.left_bumper == true){
                robot.extendintake(1250);
            }
            if(this.gamepad2.right_bumper == true){
                //robot.home();
            }
            if(this.gamepad1.dpad_up == true) {
                robot.moveF( 0.5,10);
            }

            if(this.gamepad1.dpad_down == true) {
                robot.moveB(0.5, 10);
            }
            if(this.gamepad2.dpad_left == true) {
                robot.dropdown( 10);
            }

            if(this.gamepad2.dpad_right == true) {
                robot.dropup(10);
            }
            if(this.gamepad1.dpad_left == true) {
                robot.moveR( 0.3, 10);
            }

            if(this.gamepad1.dpad_right == true) {
                robot.moveL(0.3, 10);
            }
            if(this.gamepad1.a == true){
                robot.arcTurn(-1, 900);
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