package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(group = "Tau", name = "Auto Crater IMU")
//@Disabled
public class AUTO_CRATER_IMU extends AUTO_METHODS_IMU{

    Hardware hard = new Hardware();
    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap, telemetry); //Thanks for the code Evan

        //Call other methods
        /*while(opModeIsActive())
        {
            telemetry.addData("first angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("second angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("third angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }`*/

        unhang();
        //blockLocation = "Center";
        switch (blockLocation) {
            case "Left":
                //Drive forward 1/2 block diagonal plus an inch to get the middle of the robot to the first corner
                driveForward(0.33, Math.sqrt(2) * 12 -2);



                //turn 45 degrees to the left mineral
                turnDegrees(0.33, 45);


                //drive into the mineral, knocking it off
                driveForward(0.33, 16);      //21


                //turn arctan(1.0) degrees because we want to go to the middle of the adjacent block
                turnDegrees(0.33, Math.toDegrees(Math.atan(1.0)));


                //drive forward 1/2 a diagonal of a block
                driveForward(0.33, Math.sqrt(2) * 12 + 7);


                //turn towards the depot
                turnDegrees(0.33, 43);


                //drive to the depot plus a foot-ish in order to leave the block in the depot
                driveForward(0.33, 63);


                //drive BACKWARDS a foot so we don't move the block when we turn
                driveForward(0.33, -9);


                //turn 90 degrees to set up the mineral arm
                turnDegrees(0.33, 90);

                //arm method called here

                //drop the mineral arm
                dropArm();

                //turn back towards the crater, turning a little more than 90 degrees to account for weird starting motion
                turnDegrees(0.33, 110);


                //lower the lift so we don't tip over when we go into the crater
                dropLift();

                //drive forward into the crater by driving forward until a certain angle is met with the IMU
                //then waiting 1/2 a second to get fully into the crater
                //driveForward(0.25, 81);
                driveForwardToCrater(0.25, 81);

                break;
            case "Right":
                driveForward(0.33, Math.sqrt(2) * 12 - 2);

                turnDegrees(0.33, -40);

                driveForward(0.33, 16);

                turnDegrees(0.33, -30);

                turnDegrees(0.33, 30);

                driveForward(0.33, -24);

                turnDegrees(0.33, 100);
                //sleepTau(1000);
                driveForward(0.33, Math.sqrt(2) * 24);
                //sleepTau(2000);
                turnDegrees(0.33, 78);
                //sleepTau(1250);
                driveForward(0.33, 45);
                //sleepTau(3000);
                turnDegrees(0.33, 90);
                //sleepTau(1000);
                dropArm();
                turnDegrees(0.33, 115);
                //sleepTau(1500);
                dropLift();
                //driveForward(0.25, 65);
                driveForwardToCrater(0.25, 65);
                //sleepTau(5000);
                break;
            default: //Center
                driveForward(0.33, Math.sqrt(2) * 24 + 1);
                //sleepTau(1750);
                driveForward(0.33, -Math.sqrt(2) * 12 - 1);
                //sleepTau(1000);
                turnDegrees(0.33, 69);
                //sleepTau(1000);
                driveForward(0.33, Math.sqrt(2) * 36 - 13);
                //sleepTau(2000);
                turnDegrees(0.33, 65);
                //sleepTau(1500);
                driveForward(0.33, 50);
                //sleepTau(3000);
                turnDegrees(0.33, 90);
                //sleepTau(1000);
                dropArm();
                turnDegrees(0.33, 110);
                //sleepTau(1000);
                dropLift();
                //driveForwardAndDropLift( 78);
                //driveForward(0.25, 75);
                driveForwardToCrater(0.25, 75);
                //sleepTau(5000);
                break;
        }

        //hard.turnOffFlash();
        //deactivate the TensorFlow library to free up system resources
        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
    }
}
