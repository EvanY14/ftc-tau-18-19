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
        switch (blockLocation) {
            case "Left":
                //Drive forward 1/2 block diagonal plus an inch to get the middle of the robot to the first corner
                driveForward(0.25, Math.sqrt(2) * 12 -2);


                //turn 45 degrees to the left mineral
                turnDegrees(0.25, 45);


                //drive into the mineral, knocking it off
                driveForward(0.25, 16);      //21


                //turn arctan(1.0) degrees because we want to go to the middle of the adjacent block
                turnDegrees(0.25, Math.toDegrees(Math.atan(1.0)));


                //drive forward 1/2 a diagonal of a block
                driveForward(0.3, Math.sqrt(2) * 12 + 6);


                //turn towards the depot
                turnDegrees(0.25, 45);


                //drive to the depot plus a foot-ish in order to leave the block in the depot
                driveForward(0.25, 63);


                //drive BACKWARDS a foot so we don't move the block when we turn
                driveForward(0.25, -9);


                //turn 90 degrees to set up the mineral arm
                turnDegrees(0.25, 90);

                //arm method called here

                //drop the mineral arm
                dropArm();

                //turn back towards the crater, turning a little more than 90 degrees to account for weird starting motion
                turnDegrees(0.25, 88);


                //lower the lift so we don't tip over when we go into the crater
                dropLift();

                //drive forward into the crater by driving forward until a certain angle is met with the IMU
                //then waiting 1/2 a second to get fully into the crater
                driveForward(0.5, 81);

                break;
            case "Right":
                driveForward(0.25, Math.sqrt(2) * 12 - 2);

                turnDegrees(0.25, -40);

                driveForward(0.25, 16);

                turnDegrees(0.25, -30);

                turnDegrees(0.25, 30);

                driveForward(0.25, -24);

                turnDegrees(0.25, 90);
                //sleepTau(1000);
                driveForward(0.25, Math.sqrt(2) * 24);
                //sleepTau(2000);
                turnDegrees(0.25, 75);
                //sleepTau(1250);
                driveForward(0.25, 45);
                //sleepTau(3000);
                turnDegrees(0.25, 90);
                //sleepTau(1000);
                dropArm();
                turnDegrees(0.25, 90);
                //sleepTau(1500);
                dropLift();
                driveForward(0.5, 65);
                //sleepTau(5000);
                break;
            case "Center":
                driveForward(0.25, Math.sqrt(2) * 24 + 1);
                //sleepTau(1750);
                driveForward(0.25, -Math.sqrt(2) * 12 - 1);
                //sleepTau(1000);
                turnDegrees(0.25, 65);
                //sleepTau(1000);
                driveForward(0.25, Math.sqrt(2) * 36 - 13);
                //sleepTau(2000);
                turnDegrees(0.25, 65);
                //sleepTau(1500);
                driveForward(0.25, 50);
                //sleepTau(3000);
                turnDegrees(0.25, 90);
                //sleepTau(1000);
                dropArm();
                turnDegrees(0.25, 100);
                //sleepTau(1000);
                dropLift();
                driveForward(0.5, 78);
                //sleepTau(5000);
                break;
        }

        //deactivate the TensorFlow library to free up system resources
        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
    }
}
