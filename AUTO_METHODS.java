package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;

/**
 * Created by Evan Yu on 9/16/2018.
 */

class AUTO_METHODS extends LinearOpMode {
    Hardware robot = new Hardware();

    //org.firstinspires.ftc.teamcode.Vision vision = new org.firstinspires.ftc.teamcode.Vision();

    private double leftSpeed = 0;
    private double rightSpeed = 0;


    ElapsedTime period = new ElapsedTime();

    private ArrayList<Double> location = new ArrayList<Double>();
    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private Position position;

    private int backLeftMotorPosition = 0;
    private int backRightMotorPosition = 0;
    private int frontLeftMotorPosition = 0;
    private int frontRightMotorPosition = 0;

    //Use if all motor positions should be the same
    private int motorPosition = 0;

    public AUTO_METHODS(){}


    //static{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
    //public void runOpMode() throws InterruptedException {}
    /*@Override
    public void init(){}

    @Override
    public void loop(){}*/
    public void hardwareMapPrint(){
        telemetry.addData("HardwareMap",hardwareMap);
        telemetry.update();
    }
    public void setUp(){
        telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        telemetry.update();
//clickity clackity
        robot.init(hardwareMap);
        // Set up our telemetry dashboard
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();
    }
    //Behind the scenes methods

    //Sets speeds of motors
    private void speed(double speed){
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
    }


    //Auto methods to call


   /* public void getLocationOnField() {
        location.set(0, vision.getRobotX());
        location.set(1, vision.getRobotY());
        location.set(2, vision.getRobotZ());
        location.set(3, vision.getRobotHeading());
        telemetry.addData("Location", "X:" + location.get(0) + "," + "Y:" + location.get(1) + "Z:" + location.get(2));
    }*/

    /*public void unhang() {
        //time it takes to drop robot
        long dropTime = 0;
        robot.leftLiftMotor.setPower(-1);
        robot.leftLiftMotor.setPower(-1);
        try {
            robot.sleepTau(dropTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.leftLiftMotor.setPower(0);
        robot.rightLiftMotor.setPower(0);
    }*/

    //drive forward certain distance at certain speed(speed should be no more than 1), distance is in inches
    public void driveForward(double speed, double distance){
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        speed(speed);
        motorPosition += (int)((distance / (6 * Math.PI)) * 1120);
        telemetry.addData("Motor position", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("Right motor position", robot.backRightMotor.getCurrentPosition());
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()- motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()- motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        telemetry.addData("Motor position", robot.backLeftMotor.getCurrentPosition());
        telemetry.addData("Right motor position", robot.backRightMotor.getCurrentPosition());
    }

    public void turnDegrees(double speed, double degree){
        speed(speed);

    }
    /*public void scanMinerals(){
        if(vision.seesSilver()){

        }
    }*/
    public void sleepTau(long milliSec){try{Thread.sleep(milliSec);Log.i(TAG,"IMU Heading: "+imu.getAngularOrientation().firstAngle);}catch(InterruptedException e){throw new RuntimeException(e);}}

    @Override
    public void runOpMode() throws InterruptedException {}
}