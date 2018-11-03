package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Tau", name = "Auto Crater")
public class AUTO_CRATER extends AUTO_METHODS{

    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap); //Thanks for the code Evan

        //Call other methods
        unhang();
        driveForward(0.5, Math.sqrt(2) * 12);
        sleepTau(2000);
        turnDegrees(0.5, 45);
        sleepTau(2000);
        driveForward(0.5, 18);      //21
        sleepTau(2000);
        turnDegrees(0.5, 100/*Math.atan(7.0)*/);
        sleepTau(2000);
        driveForward(0.5, 66);
        sleepTau(2000);
        turnDegrees(0.5, 90);
        sleepTau(2000);
        //arm method called here
        turnDegrees(0.5, 100);
        sleepTau(2000);
        driveForward(0.75, 70);
        sleepTau(5000);

    }
}
