package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue depot", group = "Tau")
public class AUTO_DEPOT extends AUTO_METHODS {

    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap);

        //Call other methods
        unhang();
        driveForward(0.5, Math.sqrt(2) * 24);
        sleepTau(2000);
        turnDegrees(0.5, 45);
        sleepTau(2000);
        driveForward(0.5, 4.5 );
        sleepTau(2000);
        //calls arm method
        turnDegrees(0.5, 90);
        sleepTau(2000);
        driveForward(0.5, 3);
        sleepTau(2000);
        turnDegrees(0.5, 90);
        sleepTau(2000);
        driveForward(0.75, 84);
        sleepTau(5000);


    }

}
