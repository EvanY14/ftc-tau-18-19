package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Tau", name = "Auto Crater Hardcode")
public class AUTO_CRATER_HARDCODE extends AUTO_METHODS_HARDCODE{

    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap, telemetry); //Thanks for the code Evan

        //Call other methods

        unhang();
        if(blockLocation.equals("Left")) {
            driveForward(0.25, Math.sqrt(2) * 12 + 1);
            sleepTau(1500);
            turnDegrees(0.25, 45);
            sleepTau(1500);
            driveForward(0.25, 16);      //21
            sleepTau(2000);
            turnDegrees(0.25, Math.toDegrees(Math.atan(1.0)));
            sleepTau(1500);
            driveForward(0.3, Math.sqrt(2) * 12 + 4);
            sleepTau(1000);
            turnDegrees(0.25, 47.5);
            sleepTau(750);
            driveForward(0.25, 63);
            sleepTau(3000);
            driveForward(0.25, -12);
            sleepTau(1000);
            turnDegrees(0.25, 90);
            sleepTau(2000);
            //arm method called here
            dropArm();
            turnDegrees(0.25, 100);
            sleepTau(2000);
            dropLift();
            driveForward(0.5, 84);
            sleepTau(5000);
        } else if(blockLocation.equals("Right")){
            driveForward(0.25, Math.sqrt(2) * 12 + 1);
            sleepTau(1000);
            turnDegrees(0.25, -45);
            sleepTau(750);
            driveForward(0.25, 16);
            sleepTau(1000);
            turnDegrees(0.25, -30);
            sleepTau(750);
            turnDegrees(0.25, 30);
            sleepTau(750);
            driveForward( 0.25, -24);
            sleepTau(1000);
            turnDegrees(0.25,120);
            sleepTau(1000);
            driveForward(0.25,Math.sqrt(2) * 24);
            sleepTau(2000);
            turnDegrees(0.25, 67.5);
            sleepTau(1250);
            driveForward(0.25,42);
            sleepTau(3000);
            turnDegrees(0.25, 90);
            sleepTau(1000);
            dropArm();
            turnDegrees(0.25, 105);
            sleepTau(1500);
            driveForwardAndDropLift(81);
        }else if (blockLocation.equals("Center")){
            driveForward(0.25, Math.sqrt(2) * 24 + 2);
            sleepTau(1750);
            driveForward(0.25, -Math.sqrt(2) * 12 - 2);
            sleepTau(1000);
            turnDegrees(0.25, 75);
            sleepTau(1000);
            driveForward(0.25, Math.sqrt(2) * 36 - 10);
            sleepTau(2000);
            turnDegrees(0.25, 65);
            sleepTau(1500);
            driveForward(0.25, 40);
            sleepTau(2000);
            turnDegrees(0.25, 90);
            sleepTau(1000);
            dropArm();
            turnDegrees(0.25, 110);
            sleepTau(1000);
            dropLift();
            driveForward(0.25, 78);
            sleepTau(5000);
        }

    }
}
