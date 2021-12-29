package org.firstinspires.ftc.teamcode;
//Fix if detecting 2 or 0 minerals
//Give Power to Servo Motor holder
//Buttons to move latch and slide
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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

    final double DESIRED_DISTANCE = 8.0; //  this is how close the camera should get to the target (inches)
    //  The GAIN constants set the relationship between the measured position error,
    //  and how much power is applied to the drive motors.  Drive = Error * Gain
    //  Make these values smaller for smoother control.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MM_PER_INCH = 25.40 ;   //  Metric conversion

    private static final String VUFORIA_KEY = "AV8zEej/////AAABmVo2vNWmMUkDlkTs5x1GOThRP0cGar67mBpbcCIIz/YtoOvVynNRmJv/0f9Jhr9zYd+f6FtI0tYHqag2teC5GXiKrNM/Jl7FNyNGCvO9zVIrblYF7genK1FVH3X6/kQUrs0vnzd89M0uSAljx0mAcgMEEUiNOUHh2Fd7IOgjlnh9FiB+cJ8bu/3WeKDxnDdqx6JI5BlQ4w7YW+3X2icSRDRlvE4hhuW1VM1BTPQgds7OtHKqUn4Z5w1Wqg/dWiOHxYTww28PVeg3ae4c2l8FUtE65jr2qQdQNc+DMLDgnJ0fUi9Ww28OK/aNrQQnHU97TnUgjLgCTlV7RXpfut5mZWXbWvO6wA6GGkm3fAIQ2IPL";

    VuforiaLocalizer vuforia    = null;
    OpenGLMatrix targetPose     = null;
    String targetName           = "";


    @Override
    public void runOpMode()  {
        org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot (hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData ("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();
        //Driver must press INIT and then ▶️

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

        //use skystone target image instead
        // VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        // VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        //stoneTarget.setName("Stone Target");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();
        //targetsSkyStone.activate();



        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        boolean targetFound     = false;    // Set to true when a target is detected by Vuforia
        double  targetRange     = 0;        // Distance from camera to target in Inches
        double  targetX         = 0;        // X Distance
        double  targetY         = 0;        // Y Distance
        double  targetBearing   = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.
        double  drive           = 0;        // Desired forward power (-1 to +1)
        double  turn            = 0;        // Desired turning power (-1 to +1)

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();

            /********** GamePad 1 ****************/

            // Log outtake encoder values
            if (this.gamepad1.x) {
                robot.logOuttakeEncoders();
            }

            // Reset outtake encoder values
            if (this.gamepad1.y) {
                robot.resetOuttakeEncoders();
            }

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
//            if (this.gamepad1.right_stick_y > 0.5) {
//                robot.moveF(1, 10);
//            }
//
//
//            if (this.gamepad1.right_stick_y < -0.5) {
//                robot.moveB(1, 10);
//            }
//
//            if (this.gamepad1.right_stick_x > 0.5) {
//                robot.moveR(1, 10);
//            }
//
//            if (this.gamepad1.right_stick_x < -0.5) {
//                robot.moveL(1, 10);
//            }

            if (this.gamepad1.right_stick_y >= Math.abs(this.gamepad1.right_stick_x) ||
                this.gamepad1.right_stick_y <= (-1) * Math.abs(this.gamepad1.right_stick_x)) {
                robot.setMotorPowers(
                        (0.5) * this.gamepad1.right_stick_y,
                        (-0.5) * this.gamepad1.right_stick_y,
                        (0.5) * this.gamepad1.right_stick_y,
                        (-0.5) * this.gamepad1.right_stick_y
                );
            }

            if (this.gamepad1.right_stick_x >= Math.abs(this.gamepad1.right_stick_y) ||
                    this.gamepad1.right_stick_x <= (-1) * Math.abs(this.gamepad1.right_stick_y)) {
                robot.setMotorPowers(
                        (0.8) * this.gamepad1.right_stick_x,
                        (0.8) * this.gamepad1.right_stick_x,
                        (-0.8) * this.gamepad1.right_stick_x,
                        (-0.8) * this.gamepad1.right_stick_x
                );
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
                Thread perfect = new Thread() {
                    public void run() {
                        robot.perfectDropEncoder();
                    }
                };

                perfect.start();

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
//                robot.vumarkDetection(targetsFreightFrenzy, MM_PER_INCH);
                robot.carouselmove(1500);
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
//            if(this.gamepad1.a == true){
//                robot.arcTurn(-1, 900);
//            }

            if(this.gamepad2.right_stick_x > 0.5){
            }
            if(this.gamepad2.right_stick_x < -0.5){
            }
            /****************** GamePad 2 **************/
            // Pincher

        };
    };
}