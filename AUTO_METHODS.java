package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.teamcode.Vision1;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;
import java.util.Locale;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.TAG;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * Created by Evan Yu on 9/16/2018.
 */

public class AUTO_METHODS extends LinearOpMode {
    Hardware robot = new Hardware();

    //Vision1 vision = new Vision1();

    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private VectorF translation;
    private Orientation rotation;

    ElapsedTime period = new ElapsedTime();

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    //VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private ArrayList<Double> location = new ArrayList<Double>();

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private Position position;

    private int backLeftMotorPosition = 0;
    private int backRightMotorPosition = 0;
    private int frontLeftMotorPosition = 0;
    private int frontRightMotorPosition = 0;
    private double vuMarkEnd = 0;

    private final double ticksPerRotation = 1120;
    private final double robotRotationRadius = 6.5;
    //Use if all motor positions should be the same
    private int motorPosition = 0;

    /*
    Vision variables
     */

    public AUTO_METHODS(){}


    //static{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
    //public void runOpMode() throws InterruptedException {}
    /*@Override
    public void init(){}


    @Override
    public void loop(){}*/
    public void hardwareMapPrint(){
        telemetry.addData("HardwareMap",hardwareMap);
        telemetry.update();
    }

    public void setUp(HardwareMap hwMap){
        telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        telemetry.update();
//clickity clackity
        robot.init_auto(hwMap);
        //robot.imageTrackables.activate();
        // Set up our telemetry dashboard
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();
    }
    //Behind the scenes methods

    //Sets speeds of motors
    private void speed(double speed){
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
    }


    /*Auto methods to call
      Right is forwards, left is backwards
      All distances have to be multiplied by ticksPerRotation and divided by 6 * Pi
     */


   public void getLocationOnField() {
       telemetry.addData("Status:", "About to run opmode");
       getLocation();
       telemetry.addData("Status:", "ran opmode");
        location.set(0, getRobotX());
        location.set(1, getRobotY());
        location.set(2, getRobotZ());
        location.set(3, getRobotHeading());
        telemetry.addData("Location", "X:" + location.get(0) + "," + "Y:" + location.get(1) + "Z:" + location.get(2));
        telemetry.update();
   }

    /*public void unhang() {
        //time it takes to drop robot
        long dropTime = 0;
        robot.leftLiftMotor.setPower(-1);
        robot.leftLiftMotor.setPower(-1);
        try {
            robot.sleepTau(dropTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.leftLiftMotor.setPower(0);
        robot.rightLiftMotor.setPower(0);
    }*/

    //drive forward certain distance at certain speed(speed should be no more than 1), distance is in inches
    public void driveForward(double speed, double distance){
        speed(speed);
        motorPosition = (int)((distance / (6 * Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()- motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()- motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
    }

    public void turnDegrees(double speed, double degree){
        speed(speed);
        double distance = (degree * (2 * robotRotationRadius * Math.PI) / 360);
        motorPosition = (int)((distance / (6*Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()+ motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()+ motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
    }
    /*public void scanMinerals(){
        if(vision.seesSilver()){

        }
    }*/
    public ArrayList<Double> getLocation(){
        String Tag = "Error message";
        Log.d(Tag, "test");
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        telemetry.addData("Status:", "Starting location method");
        Log.d(Tag, "Started");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Log.d(Tag, "created cameraMonitorViewId variable");
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        Log.d(Tag, "finished start");
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AUTPgLj/////AAABmftxO0IFGU3urmaLhFDDt+04jQVVUEnMoybqfXkW+2kDybcXkSk00wQ1RARTA6i+W3x8pWjVDY/xcKrLUwZZKYSdeSlSWW+nMK4s5AEaTS8K0Re8OrF3JF3zmHz4julP101iBl7+dpVOEFw10laj2E0q0bvw9vqvXMMjg8J3zdXiDS4zzHPRl0Iwx6iaH4ZmmE4VqXiJ8kXrZ9bc897oR4FcC01mF+cX3x6oi5e8ZpQanSDPp2/IBbvUxi/oe2ImrNpZTczvZLMwYMTQqgfeN9Ewz5KtCbAwfCLARiW5QZ/EOOdlLfGIPXGYesLuVPswhWP5HCCCrberCUZ+y+2OGj7+SlesgFSD8qwWNMQh+Erx" ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = robot.vuforia.loadTrackablesFromAsset("RoverRuckus");
       // VuforiaTrackables targetsRoverRuckus = robot.imageTrackables;
        Log.d(Tag,"about to add vumarks");
        VuforiaTrackable blueRover = robot.imageTrackables.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = robot.imageTrackables.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = robot.imageTrackables.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = robot.imageTrackables.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(robot.imageTrackables);
        telemetry.addData("Status:", "Added all of the vumarks");
        Log.d(Tag, "added vumarks");
        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */
        Log.d(Tag, "Initialized vumarks");
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        /** Wait for the game to begin */
        /*telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();*/

        /** Start tracking the data sets we care about. */
        //targetsRoverRuckus.activate();
        Log.d(Tag, "About to start loop");
        telemetry.addData("Status", "begin");
        int runCounter = 0;
        while (true) {
        Log.d("Status", "in loop");
        runCounter += 1;
        Log.d("Num times run", "" + runCounter);
            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                location.add((double)(translation.get(0)/mmPerInch));
                location.add((double)(translation.get(1)/mmPerInch));
                location.add((double)(translation.get(2))/mmPerInch);
                // express the rotation of the robot in degrees.
                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                return location;
            }
            else {
                telemetry.addData("Visible Target", "none");
                if(runCounter > 500){
                    break;
                }
            }
            //telemetry.update();

        }
        return location;
    }
    public double getRobotX(){
        return translation.get(0) / mmPerInch;
    }
    public double getRobotY(){
        return translation.get(1) / mmPerInch;
    }
    public double getRobotZ(){
        return translation.get(2) / mmPerInch;
    }
    public double getRobotRoll(){
        return rotation.firstAngle;
    }
    public double getRobotPitch(){
        return rotation.secondAngle;
    }
    public double getRobotHeading(){
        return rotation.thirdAngle;
    }
    public void sleepTau(long milliSec){try{Thread.sleep(milliSec);}catch(InterruptedException e){throw new RuntimeException(e);}}
    
    public void dropArm() {
    	robot.markerClawServo.setPosition(95);
    }
    
    public void raiseArm() {
    	robot.markerClawServo.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}