package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.teamcode.Vision1;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//import org.opencv.android.*;

/**
 * Created by Evan Yu on 9/16/2018.
 */

public class AUTO_METHODS extends LinearOpMode {
    Hardware robot = new Hardware();

    //Vision1 vision = new Vision1();
    //ImageProcessor.State blockPosition;

    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private VectorF translation;
    private Orientation rotation;
    private Position initialPos = null;
    private double initialAngle = 0;
    private double finalAngle = 0;
    private double finalX = 0;
    private double finalY = 0;
    ElapsedTime period = new ElapsedTime();

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    //VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private String VUFORIA_KEY = "AUTPgLj/////AAABmftxO0IFGU3urmaLhFDDt+04jQVVUEnMoybqfXkW+2kDybcXkSk00wQ1RARTA6i+W3x8pWjVDY/xcKrLUwZZKYSdeSlSWW+nMK4s5AEaTS8K0Re8OrF3JF3zmHz4julP101iBl7+dpVOEFw10laj2E0q0bvw9vqvXMMjg8J3zdXiDS4zzHPRl0Iwx6iaH4ZmmE4VqXiJ8kXrZ9bc897oR4FcC01mF+cX3x6oi5e8ZpQanSDPp2/IBbvUxi/oe2ImrNpZTczvZLMwYMTQqgfeN9Ewz5KtCbAwfCLARiW5QZ/EOOdlLfGIPXGYesLuVPswhWP5HCCCrberCUZ+y+2OGj7+SlesgFSD8qwWNMQh+Erx";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



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
    private int markerGrabber = 0;
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

    public void setUp(HardwareMap hwMap, Telemetry telemetry){
        telemetry.addData("Readiness", "NOT READY TO START, PLEASE WAIT");
        telemetry.update();
//clickity clackity
        robot.init_auto(hwMap, telemetry);
        boolean useFullRes = true;
        Context context = hardwareMap.appContext;
        //cameraManager.initialize(context, useFullRes, this);
        //imageProcessor.initialize(useFullRes, this, true, cameraManager.height, cameraManager.width);
        //robot.imageTrackables.activate();
        // Set up our telemetry dashboard
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Readiness", "Press Play to start");
        telemetry.update();

        // Wait until we're told to go
        waitForStart();
        robot.tfod.activate();
    }
    //Behind the scenes methods

    //Sets speeds of motors
    private void speed(double speed){
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
    }

    public void speedLift(double speed){
        robot.rightLiftMotor.setPower(speed);
        robot.leftLiftMotor.setPower(speed);
    }

    /*Auto methods to call
      Right is forwards, left is backwards
      All distances have to be multiplied by ticksPerRotation and divided by 6 * Pi
     */
    public void hang(){
        ArrayList<Double> navigate = new ArrayList<>();
        ArrayList<Double> location = getLocation();
        navigate.add(location.get(0) > 0 ? 36.0:-36.0);
        navigate.add(location.get(1) > 0 ? 36.0:-36.0);
        navigateTo(location);
        turnDegrees(0.5, getRobotHeading() + 45);
        driveForward(0.5, -2 * Math.sqrt(2) * 12);
    }

    /*public ImageProcessor.State getBlockLocation(){
        imageProcessor.takePicture();
        blockPosition = imageProcessor.blockState;
        blockPosition = ImageProcessor.State.BLOCK_IS_LEFT;
        telemetry.addData("Block position", blockPosition);

        return blockPosition;
    }*/

   /* public void knockBlockOff(ImageProcessor.State blockPosition){
        if(blockPosition == ImageProcessor.State.BLOCK_IS_CENTER){
            driveForward(0.5, Math.sqrt(2) * 12 + 3);
            sleepTau(1200);
            turnDegrees(0.5, -135);
            sleepTau(1000);
            driveForward(0.5, 12);
            sleepTau(750);
            turnDegrees(0.5, 45);
            sleepTau(1000);
        }
    }*/
    //use inches with coordinates
    public void navigateTo(ArrayList<Double> location){
        double heading = getRobotHeading();
        turnDegrees(0.5, -getRobotHeading());
        turnDegrees(0.5, -90 - Math.toDegrees(Math.atan(Math.abs(getRobotY() - location.get(1))/Math.abs(getRobotX() - location.get(0)))));
        driveForward(0.5, Math.sqrt(Math.pow(getRobotY() - location.get(1), 2) + Math.pow(getRobotX() - location.get(0),2)));

    }

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

    public void dropArm() {
        robot.markerArm.setPosition(0.5);
        sleepTau(1500);
        robot.markerArm.setPosition(0);
        sleepTau(1500);
    }

    public void unhang() {
        robot.stopper.setPosition(0.95);
        telemetry.addData("Status", "About to wait 5 sec");
        telemetry.update();
        sleepTau(2000);
        telemetry.addData("Status", "done");
        telemetry.update();
        speedLift(1);
        robot.leftLiftMotor.setTargetPosition((int)robot.leftLiftMotor.getCurrentPosition() + 5700);
        robot.rightLiftMotor.setTargetPosition((int)robot.rightLiftMotor.getCurrentPosition() + 5700);
        sleepTau(3000);
    }

    public void dropLift(){
        speedLift(1);
        robot.leftLiftMotor.setTargetPosition(robot.rightLiftMotor.getCurrentPosition() - 5600);
        robot.rightLiftMotor.setTargetPosition(robot.leftLiftMotor.getCurrentPosition() - 5600);
        sleepTau(3000);
    }
    //drive forward certain distance at certain speed(speed should be no more than 1), distance is in inches
    public void driveForward(double speed, double distance){
        speed(speed);
        /*initialPos = robot.imu.getPosition();
        double heading = robot.imu.getAngularOrientation().firstAngle;
        double deltaX = Math.sin(heading) * distance;
        double deltaY = Math.cos(heading) * distance;
        finalX = initialPos.x + deltaX;
        finalY = initialPos.y;*/
        motorPosition = (int)((distance / (6 * Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()- motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()- motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        /*while(true){
            if(Math.abs(robot.imu.getPosition().x) >= Math.abs(finalX) && Math.abs(robot.imu.getPosition().y) >= Math.abs(finalY)){
                speed(0);
                sleepTau(500);
                break;
            }
        }*/

    }

    public void turnDegrees(double speed, double degree){
        speed(speed);
        /*initialAngle = robot.imu.getAngularOrientation().firstAngle;
        finalAngle = initialAngle + degree;*/
        double distance = (degree * (2 * robotRotationRadius * Math.PI) / 360);
        motorPosition = (int)((distance / (6*Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()+ motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()+ motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        /*while(true){
            if(Math.abs(robot.imu.getAngularOrientation().firstAngle) >= Math.abs(finalAngle)){
                speed(0);
                sleepTau(500);
                break;
            }
        }*/
    }
    /*public void scanMinerals(){
        if(vision.seesSilver()){

        }
    }*/
    public String getBlockLocation(){
        String location = "";
        /*robot.initVuforia(hwMap);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.initTfod(hwMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
                /** Activate Tensor Flow Object Detection. */

            while (opModeIsActive()) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();
                                    location = "Left";
                                    break;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    location = "Right";
                                    break;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    location = "Center";
                                    break;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
            //return "tfod null";
        }
        return location;
    }

    public void knockBlockOff(String block){
        driveForward(0.5, Math.sqrt(2) * 12);
        sleepTau(750);
        if(block.equals("Left") || block.equals("Right")){
            turnDegrees(0.5, block.equals("Left") ? 45: -45);
            sleepTau(1000);
            driveForward(0.5, 23.5);
            sleepTau(2000);
            turnDegrees(0.5, block.equals("Left") ? -90: 90);
            sleepTau(1000);
            driveForward(0.5, 22);
            sleepTau(2000);
            if(block.equals("Left")){
                turnDegrees(0.5, 90);
                sleepTau(1000);
                dropArm();
                turnDegrees(0.5, 75);
                sleepTau(750);
                driveForward(0.5, Math.sqrt(2) * 9 + 1);
                sleepTau(2000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                //calls arm method
                dropLift();
                driveForward(1, 55);
                sleepTau(5000);
            }else{

                dropArm();
                turnDegrees(0.5, 45);
                sleepTau(750);
                driveForward(0.5, Math.sqrt(2) * 11);
                sleepTau(2000);
                turnDegrees(.5, 40);
                sleepTau(1000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                //calls arm method
                dropLift();
                driveForward(1, 55);
                sleepTau(5000);
            }

        } else if(block.equals("Center")){
            driveForward(0.5, 2 * Math.sqrt(2) * 12);
            sleepTau(1500);
            turnDegrees(0.5, 90);
            sleepTau(750);
        }
    }

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
        final int CAMERA_FORWARD_DISPLACEMENT  = -222;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 76;   // eg: Camera is 200 mm above ground
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
    public double getRobotX(){return getLocation().get(0) / mmPerInch; }
    public double getRobotY(){return getLocation().get(1) / mmPerInch; }
    public double getRobotZ(){
        return getLocation().get(2) / mmPerInch;
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

    @Override
    public void runOpMode() throws InterruptedException {}

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(HardwareMap hwMap) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}