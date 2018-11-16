package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        setUp(hardwareMap, telemetry);
        while(opModeIsActive()){
            telemetry.addData("Location" , getBlockLocation());
            telemetry.update();
            break;
        }

        //telemetry.addData("block location", getBlockLocation());
        /*sleepTau(10000);
        getLocationOnField();
        driveForward(0.5, 12);
        telemetry.addData("Status", "Drove forward");
        telemetry.update();
        sleepTau(2000);
        turnDegrees(0.5, 90);
        telemetry.addData("Status", "Turned 90 degrees left");
        telemetry.update();
        sleepTau(1500);
        turnDegrees(0.5,-90);
        telemetry.addData("Status", "turned 90 degrees right");
        telemetry.update();
        sleepTau(1500);
*/
    }
}
        /*driveForward(0.5, 12.0);
        sleepTau(10000);*/

