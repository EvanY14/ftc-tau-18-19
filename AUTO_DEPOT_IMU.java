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

@Autonomous(name = "Auto Depot IMU", group = "Tau")
public class AUTO_DEPOT_IMU extends AUTO_METHODS_IMU {

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
                sleepTau(2000);

                //back up half a block diagonal to leave the mineral in the depot
                driveForward(0.25, -Math.sqrt(2) * 12);
                sleepTau(1000);

                //Turn 90 degrees to set up the marker claw
                turnDegrees(0.25, 90);

                //Drop the marker in the depot
                dropArm();

                //Move five inches forward to give more room to turn so we don't knock off the silver mineral
                driveForward(0.25, 5);
                sleepTau(1000);

                //Slight turn to get to the middle of the block
                turnDegrees(0.25, 10);
                sleepTau(1000);

                //Drive forward to the middle of the block
                driveForward(0.25, 16);
                sleepTau(1000);

                //Turn towards the crater
                turnDegrees(0.25, 30);
                sleepTau(1000);

                //Lower the lift so we don't tip over on the way to the crater
                dropLift();

                //Drive forward until we detect that we are on the crater
                driveForwardToCrater();
                break;
            case ("Left"):
                telemetry.addData("Block position", "Left");
                telemetry.update();
                //drive forward half a block diagonal to get to the first intersection of blocks
                driveForward(0.25, Math.sqrt(2) * 12);
                sleepTau(1000);

                //turn 45 degrees to the gold mineral on the left
                turnDegrees(0.25, 45);
                sleepTau(1000);

                //Drive into the mineral, knocking it off
                driveForward(0.25, 25);
                sleepTau(1500);

                //Turn toward the depot
                turnDegrees(0.25,-80);
                sleepTau(750);

                //Drive to the depot
                driveForward(0.25, 36);
                sleepTau(1500);

                //drive backwards 12 inches
                driveForward(0.25, -12);
                sleepTau(750);

                //turn 85 degrees to drop off the marker
                turnDegrees(0.25, 80);
                sleepTau(1000);

                //drop off the marker
                dropArm();

                //Turn to the crater
                turnDegrees(0.25, 83);
                sleepTau(1000);

                //Drive forward about half a block
                /*driveForward(0.25, Math.sqrt(2) * 9 + 1);
                sleepTau(2000);*/

                //Stop
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);

                //Lower the lift
                dropLift();

                //Drive forward into the crater
                driveForwardToCrater();
                break;
            case("Right"):
                telemetry.addData("Block location", "Right");
                telemetry.update();
                driveForward(0.25, Math.sqrt(2) * 12);
                sleepTau(1000);
                turnDegrees(0.25,  -40);
                sleepTau(1500);
                driveForward(0.25, 25);
                sleepTau(1500);
                turnDegrees(0.25, 80);
                sleepTau(1000);
                driveForward(0.25, 31);
                sleepTau(1500);
                dropArm();
                turnDegrees(0.25, 90);
                sleepTau(1500);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                dropLift();
                driveForwardToCrater();
                break;
        }

    }

}