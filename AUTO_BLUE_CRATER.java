package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Tau", name = "Auto Blue Crater")
public class AUTO_BLUE_CRATER extends AUTO_METHODS{

    @Override
    public void runOpMode(){
        //set up methods
        setUp(hardwareMap); //Thanks for the code Evan

        //Call other methods
        driveForward(0.5, 2* Math.sqrt(2) * 12);
        sleepTau(2000);
        turnDegrees(0.5, 90);
        sleepTau(2000);
        driveForward(0.5, (2*Math.sqrt(2)*12)-9);
        sleepTau(2000);
        turnDegrees(0.5,45);
        sleepTau(2000);
        driveForward(0.75, 4.5*12);
        sleepTau(5000);
        turnDegrees(0.5,180);
        sleepTau(1500);
        /*driveForward(0.75, 8.5*12);
        sleepTau(5000);*/
    }
}
