package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.Vision1;

/**
 * Created by Evan Yu on 9/16/2018.
 */
@TeleOp(name = "Tau Teleop", group = "Tau")
public class Teleop extends OpMode{
    Hardware robot = new Hardware();
    //Log.d("Error message", "about to instantiate vision class");
    //Vision1 vision = new Vision1();
   // AUTO_METHODS auto = new AUTO_METHODS();
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
    private double maxPOWER = 1;
    private double endTimeA = 0;
    private boolean endGameSpeed = false;
    private double endTimeB = 0;


    @Override
    public void init() {
       telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        updateTelemetry(telemetry);

        robot.init(hardwareMap);
        Log.d("Error message", "initialized robot with vision");
        telemetry.addData("HardwareMap", hardwareMap);
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.addData("If you notice this", "You are COOL!!! (Charles was here)");
        updateTelemetry(telemetry);

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
        //auto.sleepTau(2000);
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

        if(gamepad1.a && robot.getTime() > endTimeA){
            endTimeA = robot.getTime() + 1;
            slowDrive = !slowDrive;
            maxPOWER = slowDrive?(2.0/3):1;

        }
        if(gamepad1.b && robot.getTime() > endTimeB){
            endTimeB = robot.getTime() + 1;
            endGameSpeed = !endGameSpeed;
            maxPOWER = endGameSpeed?0.25:1;
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

        //speeds of drive motors
        frontleftPOWER = backleftPOWER = leftGP1Y;
        frontrightPOWER = backrightPOWER = rightGP1Y;

        frontleftPOWER = maxPOWER * frontleftPOWER;
        frontrightPOWER = maxPOWER * frontrightPOWER;
        backleftPOWER = maxPOWER * backleftPOWER;
        backrightPOWER = maxPOWER * backrightPOWER;

        /**************************
         *********Gamepad 2********
         **************************/
        leftGP2Y = gamepad2.left_stick_y;
        rightGP2Y = gamepad2.right_stick_y;
        if(Math.abs(leftGP2Y) < 0.05){
            leftGP2Y = 0;
        }
        if(Math.abs(rightGP2Y) < 0.05){
            rightGP1Y = 0;
        }

        if(robot.rightLiftMotor.getCurrentPosition() < robot.rightLiftBottom && leftGP2Y < 0)
            leftGP2Y = 0;
        if(robot.rightLiftMotor.getCurrentPosition() > robot.rightLiftTop && leftGP2Y > 0)
            leftGP2Y = 0;
        if(robot.leftLiftMotor.getCurrentPosition() < robot.leftLiftBottom && leftGP2Y < 0)
            leftGP2Y = 0;
        if(robot.leftLiftMotor.getCurrentPosition() > robot.leftLiftTop && leftGP2Y > 0)
            leftGP2Y = 0;


        robot.rightLiftMotor.setPower(-leftGP2Y);
        robot.leftLiftMotor.setPower(-leftGP2Y);
        //setting powers to motors

        robot.frontRightMotor.setPower(frontrightPOWER);
        robot.frontLeftMotor.setPower(frontleftPOWER);
        robot.backRightMotor.setPower(backrightPOWER);
        robot.backLeftMotor.setPower(backleftPOWER);
        telemetry.addData("Left game pad 1 power", leftGP1Y);
        telemetry.addData("Right game pad 1 power", rightGP1Y);
        telemetry.addData("Left game pad 2 power", leftGP2Y);
        telemetry.addData("Right game pad 2 power", rightGP2Y);
        telemetry.addData("Slow drive", slowDrive);
        telemetry.addData("Endgame speed", endGameSpeed);
        telemetry.update();
    }



}


