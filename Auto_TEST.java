package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "Tau: Test", group = "AUTO_TEST")
public class Auto_TEST extends AUTO_METHODS {
//    Hardware robot = new Hardware();
    @Override
    public void runOpMode() throws InterruptedException{
        //hardwareMapPrint();
        //telemetry.addData("Status", "Started");
        //telemetry.update();
        //sleep(10000);
        //robot.setUp();
        //telemetry.addData("Status", "initialized");
        //telemetry.update();
        setUp();
        driveForward(0.5, 12);
        sleep(3000);
        telemetry.addData("Status", "Drove forawrd");
        telemetry.update();
    }
}
        /*driveForward(0.5, 12.0);
        sleepTau(10000);*/

