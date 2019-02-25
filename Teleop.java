package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import org.firstinspires.ftc.teamcode.Vision1;

/**
 * Created by Evan Yu on 9/16/2018.
 */
@TeleOp(name = "Tau Teleop", group = "Tau")
public class Teleop extends OpMode{
    Hardware robot = new Hardware();
    AUTO_METHODS_HARDCODE_ULTRASONIC auto = new AUTO_METHODS_HARDCODE_ULTRASONIC();
    ElapsedTime period = new ElapsedTime();
    AUTO_METHODS_IMU autoIMU = new AUTO_METHODS_IMU();
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
    private boolean intakeOut = false;
    private double endTimeIntake = 0;
    private double endTimeOuttake = 0;
    private double endTimeIntakeStop = 0;
    private double endTimeDDown = 0;
    private double endTimeDUp = 0;
    //true is up, false is down
    private boolean markerPos = true;
    private boolean slowArm = false;
    //Automatic arm motion variables
    private double gearRatio = 14;
    private double bottom;
    private double top;
    private final double angleToMove = 120;
    private final double ticksPerRotation = 2240;
    private double maxPowerArm = 1;

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
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.addData("If you notice this", "You are COOL!!! (Charles was here)");
        telemetry.update();

        //Initialize variables
        bottom = robot.liftRotationMotor.getCurrentPosition();
        top = bottom + ((angleToMove/360) * (ticksPerRotation*gearRatio));

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
            /*
                    Y
                  X   B
                    A

             */
        //Read controller input
        if(gamepad1.a && !gamepad1.b && robot.getTime() > endTimeA){
            endTimeA = robot.getTime() + 1;
            slowDrive = !slowDrive;
            maxPOWER = slowDrive?(3.0/4):1;

        }
        if(gamepad1.b && !gamepad1.a && robot.getTime() > endTimeB){
            endTimeB = robot.getTime() + 1;
            endGameSpeed = !endGameSpeed;
            maxPOWER = endGameSpeed?(0.5):1;
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

        if(gamepad1.a && gamepad1.b && endTimeA < robot.getTime() && endTimeB < robot.getTime()){
            double x = autoIMU.getRobotX();
            double y = autoIMU.getRobotY();
            //double heading = autoIMU.getRobotHeading();
            double[] currentLocation = {x,y};
            double[] dropOffLocation = autoIMU.getClosestDropOffSilver(currentLocation);
            autoIMU.driveTo(currentLocation,dropOffLocation);
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
           // leftGP2Y = 0;

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
            if(rightGP2Y < 0)
                liftExtensionPower = rightGP2Y;
            else if(rightGP2Y > 0)
                liftExtensionPower = rightGP2Y;
            else
                liftExtensionPower = 0;
        }else{
            liftExtensionPower = 0;
        }

        //Using the triggers, rotate the arm
        if((Math.abs(gamepad2.left_trigger) > 0.05)){
            liftRotationPower = gamepad2.left_trigger;
        }else if(Math.abs(gamepad2.right_trigger) > 0.05){
           liftRotationPower = -gamepad2.right_trigger;
        } else{
            liftRotationPower = 0;
        }


        //intake with 'a' button, reverse with 'x' button
        if(gamepad2.a && endTimeIntake < robot.getTime()){
            endTimeIntake = robot.getTime() + 0.25;
            robot.intakeServo.setPower(0.8);
        }else if(gamepad2.x && endTimeOuttake < robot.getTime()){
            endTimeOuttake = robot.getTime() + 0.25;
            robot.intakeServo.setPower(-0.8);
        }else if(gamepad2.b && endTimeIntakeStop < robot.getTime()){
            endTimeIntakeStop = robot.getTime() + 0.25;
            robot.intakeServo.setPower(0);
        }

        if(gamepad2.dpad_up && !robot.liftRotationMotor.isBusy() && endTimeDUp < robot.getTime()){
            robot.liftRotationMotor.setTargetPosition((int)top);
            robot.liftRotationMotor.setPower(1);
            endTimeDUp = robot.getTime() + 0.25;
        }

        if(gamepad2.dpad_down && endTimeDDown < robot.getTime()){
            slowArm = !slowArm;
            maxPowerArm = slowArm ? 0.33:1;
            endTimeDDown = robot.getTime() + 0.25;
        }




        //setting powers to motors
        robot.frontRightMotor.setPower(frontrightPOWER);
        robot.frontLeftMotor.setPower(frontleftPOWER);
        robot.backRightMotor.setPower(backrightPOWER);
        robot.backLeftMotor.setPower(backleftPOWER);
        robot.liftRotationMotor.setPower(liftRotationPower * maxPowerArm);
        robot.liftExtensionMotor.setPower(-liftExtensionPower);
        robot.liftExtensionMotor2.setPower(liftExtensionPower);
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
        telemetry.update();
    }



}


