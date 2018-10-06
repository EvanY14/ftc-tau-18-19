package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Evan Yu on 9/16/2018.
 */

public class Hardware {
    //Drive Motors******************
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    //******************************

    //Lift Motors*******************
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    //******************************

    //Servos************************
    //******************************

    //IMU***************************
    //******************************

    //Vision************************
    //******************************

    //Other*************************
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();
    //******************************

    public Hardware(){
        hwMap = null;
    }
    public void init(HardwareMap hwMap){

        this.hwMap = hwMap;
        period.reset();
        period.startTime();
        //init drive motors
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("back_left");
        backRightMotor = hwMap.dcMotor.get("back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        //init lift motors
        leftLiftMotor = hwMap.dcMotor.get("left_lift");
        rightLiftMotor = hwMap.dcMotor.get("right_lift");

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);

    }

    public double getTime(){
        return period.time();
    }

    public void sleepTau(long millis) throws InterruptedException {
        period.wait(millis);
    }
}