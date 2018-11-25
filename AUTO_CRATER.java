package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Tau", name = "Auto Crater")
public class AUTO_CRATER extends AUTO_METHODS{

    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap, telemetry); //Thanks for the code Evan

        //Call other methods

        unhang();
        driveForward(0.5, Math.sqrt(2) * 12 + 1);
        sleepTau(1500);
        turnDegrees(0.5, 45);
        sleepTau(1500);
        driveForward(0.5, 14);      //21
        sleepTau(2000);
        turnDegrees(0.5, Math.toDegrees(Math.atan(1.0)));
        sleepTau(1500);
        driveForward(0.3, Math.sqrt(2)*12 + 7);
        sleepTau(1000);
        turnDegrees(0.5,55);
        sleepTau(750);
        driveForward(0.5, 54);
        sleepTau(2000);
        turnDegrees(0.5, 90);
        sleepTau(2000);
        //arm method called here
        dropArm();
        turnDegrees(0.5, 105);
        sleepTau(2000);
        dropLift();
        driveForward(1, 84);
        sleepTau(5000);

    }
}
