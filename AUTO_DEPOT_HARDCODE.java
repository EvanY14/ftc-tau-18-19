package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name = "Auto Depot Hardcode", group = "Tau")
public class AUTO_DEPOT_HARDCODE extends AUTO_METHODS_HARDCODE {
    Hardware hard = new Hardware();
    @Override
    public void runOpMode() throws InterruptedException{
        //set up methods
        setUp(hardwareMap, telemetry);
        unhang();
        //Call other methods
        switch (blockLocation){
            case("Center"):
                telemetry.addData("Block position", "Center");
                telemetry.update();

                //Drive forward two blocks diagonals to knock off the center block and move to the depot
                driveForward(0.25, 4*Math.sqrt(2) * 12);


                //back up half a block diagonal to leave the mineral in the depot
                driveForward(0.25, -Math.sqrt(2) * 12);


                //Turn 90 degrees to set up the marker claw
                turnDegrees(0.25, 75);


                //Drop the marker in the depot
                dropArm();

                //Move five inches forward to give more room to turn so we don't knock off the silver mineral
                /*driveForward(0.25, 9);
                //sleepTau(1000);

                //Slight turn to get to the middle of the block
                turnDegrees(0.25, 7);
                //sleepTau(1000);*/

                //Drive forward to the middle of the block
                driveForward(0.25, Math.sqrt(2) * 12-6);

                //Turn towards the crater
                turnDegrees(0.25, 60);


                //Lower the lift so we don't tip over on the way to the crater
                dropLift();

                //Drive forward until we detect that we are on the crater
                //driveForwardToCrater();
                driveForward(0.5, 60);

                break;
            case ("Left"):
                telemetry.addData("Block position", "Left");
                telemetry.update();
                //drive forward half a block diagonal to get to the first intersection of blocks
                driveForward(0.25, Math.sqrt(2) * 12);

                //turn 45 degrees to the gold mineral on the left
                turnDegrees(0.25, 45);

                //Drive into the mineral, knocking it off
                driveForward(0.25, 30);

                //Turn toward the depot
                turnDegrees(0.25,-80);

                //Drive to the depot
                driveForward(0.25, 35);

                //drive backwards 12 inches
                driveForward(0.25, -12);

                //turn 85 degrees to drop off the marker
                turnDegrees(0.25, 80);

                //drop off the marker
                dropArm();

                //Turn to the crater
                turnDegrees(0.25, 83);

                //Drive forward about half a block
                /*driveForward(0.25, Math.sqrt(2) * 9 + 1);
                //sleepTau(2000);*/

                //Lower the lift
                dropLift();

                //Drive forward into the crater
                //driveForwardToCrater();
                driveForward(0.5, 78);
                break;
            case("Right"):
                telemetry.addData("Block location", "Right");
                telemetry.update();
                driveForward(0.25, Math.sqrt(2) * 12);
                turnDegrees(0.25,  -40);
                driveForward(0.25, 25);
                turnDegrees(0.25, 80);
                driveForward(0.25, 31);
                dropArm();
                turnDegrees(0.25, 85);
                dropLift();
                driveForward(0.5 , 70 );
                break;
        }

        hard.turnOffFlash();
        //deactivate the TensorFlow library to free up system resources
        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
    }

}