package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Created by Evan Yu on 9/16/2018.
 */

public class AUTO_METHODS extends LinearOpMode {
    Hardware robot = new Hardware();
    org.firstinspires.ftc.teamcode.Vision vision = new org.firstinspires.ftc.teamcode.Vision();
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    ElapsedTime period = new ElapsedTime();
    private ArrayList<Double> location = new ArrayList<Double>();

    static{System.loadLibrary("opencv_java");}

    //static{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
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