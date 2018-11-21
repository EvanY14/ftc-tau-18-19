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
            driveForward(0.3, Math.sqrt(2) * 12 + 7);
            sleepTau(1000);
            turnDegrees(0.25, 55);
            sleepTau(750);
            driveForward(0.25, 66);
            sleepTau(3000);
            driveForward(0.25, -12);
            sleepTau(1000);
            turnDegrees(0.25, 90);
            sleepTau(2000);
            //arm method called here
            dropArm();
            turnDegrees(0.25, 105);
            sleepTau(2000);
            dropLift();
            driveForward(0.25, 84);
            sleepTau(5000);
        } else if(blockLocation.equals("Right")){
            driveForward(0.25, Math.sqrt(2) * 12 + 1);
            sleepTau(1000);
            turnDegrees(0.25, -45);
            sleepTau(750);
            driveForward(0.25, 16);
            sleepTau(1000);
            turnDegrees(0.25, -20);
            sleepTau(750);
            turnDegrees(0.25, 20);
            sleepTau(750);
            driveForward( 0.25, -24);
            sleepTau(1000);
            turnDegrees(0.25,120);
            sleepTau(1000);
            driveForward(0.25,Math.sqrt(2) * 24 + Math.sqrt(2) * 12);
            sleepTau(2000);
            turnDegrees(0.25, 45);
            sleepTau(750);
            driveForward(0.25,33);
            sleepTau(2000);
            turnDegrees(0.25, 90);
            sleepTau(1000);
            dropArm();
            turnDegrees(0.25, 102.5);
            sleepTau(1500);
            dropLift();
            driveForward(0.5, 78);
            sleepTau(5000);
        }else if (blockLocation.equals("Center")){
            driveForward(0.25, Math.sqrt(2) * 24);
            sleepTau(1500);
            driveForward(0.25, -Math.sqrt(2) * 12);
            sleepTau(1000);
            turnDegrees(0.25, 75);
            sleepTau(1000);
            driveForward(0.25, Math.sqrt(2) * 36 - 5);
            sleepTau(2000);
            turnDegrees(0.25, 45);
            sleepTau(1000);
            driveForward(0.25, 33);
            sleepTau(2000);
            turnDegrees(0.25, 90);
            sleepTau(1000);
            dropArm();
            turnDegrees(0.25, 105);
            sleepTau(1000);
            dropLift();
            driveForward(0.25, 78);
            sleepTau(5000);
        }

    }
}
