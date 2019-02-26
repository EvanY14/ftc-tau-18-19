package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Vision1;

/**
 * Created by Evan Yu on 9/16/2018.
 */
@TeleOp(name = "Tau Teleop IMU", group = "Tau")
public class TeleopIMU extends OpMode{
    Hardware robot = new Hardware();
    AUTO_METHODS_HARDCODE_ULTRASONIC auto = new AUTO_METHODS_HARDCODE_ULTRASONIC();
    ElapsedTime period = new ElapsedTime();
    private BNO055IMU imu;
    //Drive variables
    private boolean slowDrive = false;
    private double leftGP1X = 0;
    private double leftGP1Y = 0;
    private double rightGP1X = 0;
    private double rightGP1Y = 0;
    private double leftGP2X = 0;
    private double leftGP2Y = 0;
    private double rightGP2X = 0;
    private double rightGP2Y = 0;
    private double GP2X = 0;
    private double GP2Y = 0;
    private double frontleftPOWER = 0;
    private double frontrightPOWER = 0;
    private double backleftPOWER = 0;
    private double backrightPOWER = 0;
    private double maxPOWER = 0.5;
    private double endTimeA = 0;
    private boolean endGameSpeed = false;
    private double endTimeB = 0;
    private double endTimeUp = 0;
    private double endTimeDown = 0;
    private double endTimeArm = 0;
    private double stopperEndTime = 0;
    private double endTimeDrive = 0;
    private double liftExtensionPower = 0;
    private double liftRotationPower = 0;
    private boolean intakeIn = false;
    private double endTimeIntake = 0;
    //true is up, false is down
    private boolean markerPos = true;

    private boolean reverseDrive = false;

    private final double markerArmdown = 0.95;
    private final double markerArmUp = 0;
    private boolean firstRun = true;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        telemetry.update();
        robot.stopper.setPosition(180);
        /*try {
            period.wait(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //`rometer bandwidth. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification
        parameters.accelRange = BNO055IMU.AccelRange.G4;
        parameters.accelBandwidth      = BNO055IMU.AccelBandwidth.HZ62_5;
        /** accelerometer power mode. See Section 3.5.2 (p27) and Section 4.2.2 (p77) of the BNO055 specification
         parameters.accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;

         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         // and named "imu".*/
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Log.d("Error message", "initialized robot with vision");
        telemetry.addData("HardwareMap", hardwareMap);
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.addData("If you notice this", "You are COOL!!! (Charles was here)");
        telemetry.update();



    }

    @Override
    public void init_loop()
    {

    }


    @Override
    public void start()
    {

    }

    @Override
    public void loop(){
        //*****************
        //Game Controller 1
        //*****************
        //Log.d("Status", "Starting loop");
        //auto.//sleepTau(2000);
        //telemetry.addData("Status", "about to get an error");

            /*
                    Y
                  X   B
                    A

             */

        //Read controller input
        //telemetry.addData("Status:", "About to get location");
        //auto.getLocationOnField();
        /*try {
            Log.d("Error message", " about to start location method in teleop class");
           auto.getLocation();
        }catch(IndexOutOfBoundsException e){
            telemetry.addData("Status:", "exception caught");
            telemetry.addData("Error", e);
            //telemetry.update();
            stop();

        }*/
        /*Log.d("Status", "finished location method and back in teleop");
        telemetry.addData("Robot x:", auto.getRobotX());
        telemetry.addData("Robot y:", auto.getRobotY());
        telemetry.addData("Robot z:", auto.getRobotZ());
        telemetry.update();*/
        /*if(firstRun){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            firstRun = false;
        }*/
        telemetry.update();
        if(gamepad1.a && robot.getTime() > endTimeA){
            endTimeA = robot.getTime() + 1;
            slowDrive = !slowDrive;
            maxPOWER = slowDrive?(2.0/3):1;

        }
        if(gamepad1.b && robot.getTime() > endTimeB){
            endTimeB = robot.getTime() + 1;
            endGameSpeed = !endGameSpeed;
            maxPOWER = endGameSpeed?(3.0/7):1;
        }

        leftGP1Y = gamepad1.left_stick_y;
        rightGP1Y = -gamepad1.right_stick_y;

        //remove slight touches
        if (Math.abs(leftGP1Y) < 0.05) {
            leftGP1Y = 0;
        }
        if (Math.abs(leftGP1X) < 0.05) {
            leftGP1X = 0;
        }

        if (Math.abs(rightGP1Y) < 0.05) {
            rightGP1Y = 0;
        }
        if (Math.abs(rightGP1X) < 0.05) {
            rightGP1X = 0;
        }

        /*if(gamepad1.dpad_up && endTimeUp < robot.getTime()){
            robot.stopper.setPosition(robot.stopper.getPosition() + 0.005);
            endTimeUp = robot.getTime() + 0.25;
        }
        if(gamepad1.dpad_down && endTimeDown < robot.getTime()){
            robot.stopper.setPosition(robot.stopper.getPosition() - 0.005);
            endTimeDown = robot.getTime() + 0.25;
        }*/

        //Pressing y on gamepad 1 moves the marker arm up or down
        //True is up, false is down
        if(gamepad1.y && endTimeArm < robot.getTime()){
            double position = !markerPos ? markerArmUp:markerArmdown;
            markerPos = !markerPos;
            robot.markerArm.setPosition(position);
            endTimeArm = robot.getTime() + 0.25;
        }

        if(gamepad1.x && endTimeDrive < robot.getTime()){
            reverseDrive = !reverseDrive;
            endTimeDrive = robot.getTime() + 0.25;
        }

        //speeds of drive motors
        frontleftPOWER = backleftPOWER = reverseDrive ? rightGP1Y:leftGP1Y;
        frontrightPOWER = backrightPOWER = reverseDrive ? leftGP1Y:rightGP1Y;

        frontleftPOWER = maxPOWER * frontleftPOWER;
        frontrightPOWER = maxPOWER * frontrightPOWER;
        backleftPOWER = maxPOWER * backleftPOWER;
        backrightPOWER = maxPOWER * backrightPOWER;

        /**************************
         *********Gamepad 2********
         **************************/
        leftGP2Y = -gamepad2.left_stick_y;
        rightGP2Y = gamepad2.right_stick_y;
        if(Math.abs(leftGP2Y) < 0.05){
            leftGP2Y = 0;
        }
        if(Math.abs(rightGP2Y) < 0.05){
            rightGP2Y = 0;
        }

        if(robot.rightLiftMotor.getCurrentPosition() > robot.rightLiftTop && leftGP2Y < 0)
            leftGP2Y = 0;

        //if(robot.leftLiftMotor.getCurrentPosition() > robot.leftLiftTop && leftGP2Y < 0)
        //leftGP2Y = 0;

        //Pressing y on gamepad 2 moves stopper up and down
        if(gamepad2.y && stopperEndTime < robot.getTime()){
            robot.stopper.setPosition(!(robot.stopper.getPosition() < 0.956) ? 0.955 : 1);
            stopperEndTime = robot.getTime() + 0.25;
        }
        if(robot.stopper.getPosition() > 0.955 && robot.stopper.getPosition() < 0.956) {
            robot.rightLiftMotor.setPower(-leftGP2Y);
            //robot.leftLiftMotor.setPower(-leftGP2Y);
        }else{
            robot.rightLiftMotor.setPower(0);
            //robot.leftLiftMotor.setPower(0);
            telemetry.addData("DRIVERS", "RAISE THE STOPPER!!!!!!!!");
            telemetry.update();
        }

        //Using the right joystick, extend/retract the arm
        //telemetry.addData("Extender pos", robot.liftExtensionMotor.getCurrentPosition());
        if(Math.abs(rightGP2Y) > 0.05){
            if(/*robot.liftExtensionMotor.getCurrentPosition() >= robot.liftMaxExtension &&*/ rightGP2Y < 0)
                liftExtensionPower = rightGP2Y;
            else if(/*robot.liftExtensionMotor.getCurrentPosition() <= 0 &&*/ rightGP2Y > 0)
                liftExtensionPower = rightGP2Y;
            else
                liftExtensionPower = 0;
        }else{
            liftExtensionPower = 0;
        }

        //Using the dpad, rotate the arm
        if((Math.abs(gamepad2.left_trigger) > 0.05)){
            liftRotationPower = -gamepad2.left_trigger;
        }else if(Math.abs(gamepad2.right_trigger) > 0.05){
           liftRotationPower = gamepad2.right_trigger;
        } else{
            liftRotationPower = 0;
        }

        //intake with a button
        if(gamepad2.a && endTimeIntake < robot.getTime()){
            endTimeIntake = robot.getTime() + 0.25;
            intakeIn = !intakeIn;
            robot.intakeServo.setPower(intakeIn?-1:0);
        }


        //setting powers to motors
        robot.frontRightMotor.setPower(frontrightPOWER);
        robot.frontLeftMotor.setPower(frontleftPOWER);
        robot.backRightMotor.setPower(backrightPOWER);
        robot.backLeftMotor.setPower(backleftPOWER);
        robot.liftRotationMotor.setPower(liftRotationPower);
        robot.liftExtensionMotor.setPower(liftExtensionPower);
        telemetry.addData("Left game pad 1 power", frontleftPOWER);
        telemetry.addData("Right game pad 1 power", frontrightPOWER);
        telemetry.addData("Actual left power", robot.frontLeftMotor.getPower());
        telemetry.addData("Actual right power", robot.frontRightMotor.getPower());
        telemetry.addData("Left game pad 2 power", leftGP2Y);
        telemetry.addData("Right game pad 2 power", rightGP2Y);
        telemetry.addData("Slow drive", slowDrive);
        telemetry.addData("Endgame speed", endGameSpeed);
        //telemetry.addData("Left lift location", robot.leftLiftMotor.getCurrentPosition());
        telemetry.addData("Stopper position", robot.stopper.getPosition());
        telemetry.addData("marker arm position", robot.markerArm.getPosition());
        telemetry.addData("Acceleration x", imu.getAcceleration().xAccel);
        telemetry.addData("Acceleration y", imu.getAcceleration().yAccel);
        telemetry.addData("Acceleration z", imu.getAcceleration().zAccel);
        Log.d("X Accel:", imu.getAcceleration().xAccel + "");
        Log.d("y Accel:", imu.getAcceleration().yAccel + "");
        Log.d("z Accel:", imu.getAcceleration().zAccel + "");
        telemetry.update();
    }



}


