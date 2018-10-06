package org.firstinspires.ftc.teamcode;

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

import java.util.ArrayList;
import java.util.Locale;
/**
 * Created by Evan Yu on 9/16/2018.
 */

public class AUTO_METHODS extends LinearOpMode {
    Hardware robot = new Hardware();
    Vision vision = new Vision();
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    ElapsedTime period = new ElapsedTime();
    private ArrayList<Double> location = new ArrayList<Double>();

    public void runOpMode() throws InterruptedException {
    }

    public void getLocationOnField() {
        location.set(0, vision.getRobotX());
        location.set(1, vision.getRobotY());
        location.set(2, vision.getRobotZ());
        location.set(3, vision.getRobotHeading());
        telemetry.addData("Location", "X:" + location.get(0) + "," + "Y:" + location.get(1) + "Z:" + location.get(2));
    }

    public void unhang() {
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
    }

    public void scanMinerals(){
        if(vision.seesSilver()){

        }
    }
}