package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Tau: Test", group = "AUTO_TEST")
public class Auto_TEST extends AUTO_METHODS_HARDCODE_ULTRASONIC {
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
        setUp(hardwareMap, telemetry);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("distance", robot.ultrasonicSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
        /*driveForward(0.5, 12.0);
        //sleepTau(10000);*/

