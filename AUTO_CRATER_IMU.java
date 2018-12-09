package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(group = "Tau", name = "Auto Crater IMU")
@Disabled
public class AUTO_CRATER_IMU extends AUTO_METHODS_IMU{

    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap, telemetry); //Thanks for the code Evan

        //Call other methods

        unhang();
        switch (blockLocation) {
            case "Left":
                //Drive forward 1/2 block diagonal plus an inch to get the middle of the robot to the first corner
                driveForward(0.25, Math.sqrt(2) * 12 + 1);
                //sleepTau(1500);

                //turn 45 degrees to the left mineral
                turnDegrees(0.25, 45);
                sleepTau(1500);

                //drive into the mineral, knowcking it off
                driveForward(0.25, 16);      //21

                //sleepTau(2000);

                //turn arctan(1.0) degrees because we want to go to the middle of the adjacent block
                turnDegrees(0.25, Math.toDegrees(Math.atan(1.0)));
                sleepTau(1500);

                //drive forward 1/2 a diagonal of a block
                driveForward(0.3, Math.sqrt(2) * 12 + 4);
                //sleepTau(1000);

                //turn towards the depot
                turnDegrees(0.25, 47.5);
                sleepTau(750);

                //drive to the depot plus a foot-ish in order to leave the block in the depot
                driveForward(0.25, 63);
                //sleepTau(3000);

                //drive BACKWARDS a foot so we don't move the block when we turn
                driveForward(0.25, -12);
                //sleepTau(1000);

                //turn 90 degrees to set up the mineral arm
                turnDegrees(0.25, 90);
                sleepTau(2000);
                //arm method called here

                //drop the mineral arm
                dropArm();

                //turn back towards the crater, turning a little more than 90 degrees to account for weird starting motion
                turnDegrees(0.25, 100);
                sleepTau(2000);

                //lower the lift so we don't tip over when we go into the crater
                dropLift();

                //drive forward into the crater by driving forward until a certain angle is met with the IMU
                //then waiting 1/2 a second to get fully into the crater
                driveForwardToCrater();
                break;
            case "Right":
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
                driveForward(0.25, -24);
                sleepTau(1000);
                turnDegrees(0.25, 120);
                sleepTau(1000);
                driveForward(0.25, Math.sqrt(2) * 24);
                sleepTau(2000);
                turnDegrees(0.25, 67.5);
                sleepTau(1250);
                driveForward(0.25, 42);
                sleepTau(3000);
                turnDegrees(0.25, 90);
                sleepTau(1000);
                dropArm();
                turnDegrees(0.25, 105);
                sleepTau(1500);
                driveForwardAndDropLift(81);
                break;
            case "Center":
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
                break;
        }

    }
}
