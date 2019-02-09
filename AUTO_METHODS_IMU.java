package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
import java.util.Locale;

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

public class AUTO_METHODS_IMU extends LinearOpMode {
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
    private double globalAngle = 0;
    private double lastAngle = 0;
    private double finalX = 0;
    private double finalY = 0;
    public String blockLocation = "";
    private double initialX;
    private double initialY;
    ElapsedTime period = new ElapsedTime();
    private final double stopper_up = 0.95;
    private final double stopper_down = 1;
    private double firstAngleZero = 0;
    private double secondAngleZero = 0;
    private double thirdAngleZero = 0;

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
    private BNO055IMU imu;
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
    private ArrayList<DcMotor> driveMotors = new ArrayList<>();
    private Orientation angles;
    private Acceleration gravity;
    private Position position;
    private Orientation angles1;
    private Acceleration gravity1;
    private Position position1;
    /*public BNO055IMU.AccelRange accelRange          = BNO055IMU.AccelRange.G4;
    /** accelerometer bandwidth. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification
    public BNO055IMU.AccelBandwidth accelBandwidth      = BNO055IMU.AccelBandwidth.HZ62_5;
    /** accelerometer power mode. See Section 3.5.2 (p27) and Section 4.2.2 (p77) of the BNO055 specification
    public BNO055IMU.AccelPowerMode accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;*/

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

    public AUTO_METHODS_IMU(){

    }


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
        //robot.init_auto_IMU(hwMap, telemetry);
        robot.init_auto(hwMap, telemetry); //wheel motors default still to RUN_TO_POSITION
        //boolean useFullRes = true;
        //Context context = hardwareMap.appContext;
        //cameraManager.initialize(context, useFullRes, this);
        //imageProcessor.initialize(useFullRes, this, true, cameraManager.height, cameraManager.width);
        //robot.imageTrackables.activate();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //`rometer bandwidth. See Section 3.5.2 (p27) and Table 3-4 (p21) of the BNO055 specification
        parameters.accelRange = BNO055IMU.AccelRange.G4;
        parameters.accelBandwidth      = BNO055IMU.AccelBandwidth.HZ62_5;
        //accelerometer power mode. See Section 3.5.2 (p27) and Section 4.2.2 (p77) of the BNO055 specification
        //parameters.accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;

         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         // and named "imu".*/
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry();

        // Start the logging of measured acceleration
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        telemetry.addData("Readiness", "Press Play to start");
        telemetry.update();

        resetAngle();

        Log.d("status", "wait for start");
        // Wait until we're told to go
        waitForStart();
        Log.d("status", "started");
        robot.tfod.activate();
        Log.d("Status", "Activated tfod");
        robot.resetTime();
        Log.d("Status", "reset time");
        robot.startTime();
        Log.d("Start Time", ""+robot.getTime());
        sleepTau(10);
    }
    //Behind the scenes methods
    //set motor to mode without encoder - used for turns using IMU
    private void setMotorIMU()
    {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void setMotorNoIMU()
    {
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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
    /*public void hang(){
        ArrayList<Double> navigate = new ArrayList<>();
        ArrayList<Double> location = getLocation();
        navigate.add(location.get(0) > 0 ? 36.0:-36.0);
        navigate.add(location.get(1) > 0 ? 36.0:-36.0);
        navigateTo(location);
        turnDegrees(0.5, getRobotHeading() + 45);
        driveForward(0.5, -2 * Math.sqrt(2) * 12);
    }*/

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
   /* public void navigateTo(ArrayList<Double> location){
        double heading = getRobotHeading();
        turnDegrees(0.5, -getRobotHeading());
        turnDegrees(0.5, -90 - Math.toDegrees(Math.atan(Math.abs(getIMUAverageYValue() - location.get(1))/Math.abs(getImuAverageXValue() - location.get(0)))));
        driveForward(0.5, Math.sqrt(Math.pow(getIMUAverageYValue() - location.get(1), 2) + Math.pow(getImuAverageXValue() - location.get(0),2)));

    }*/

    /*public void getLocationOnField() {
       telemetry.addData("Status:", "About to run opmode");
       getLocation();
       telemetry.addData("Status:", "ran opmode");
        location.set(0, getRobotX());
        location.set(1, getRobotY());
        location.set(2, getRobotZ());
        location.set(3, getRobotHeading());
        telemetry.addData("Location", "X:" + location.get(0) + "," + "Y:" + location.get(1) + "Z:" + location.get(2));
        telemetry.update();
    }*/

    public void dropArm() {
        robot.markerArm.setPosition(0.5);
        sleepTau(1500);
        robot.markerArm.setPosition(0);
        sleepTau(1500);
    }

    public void unhang() {
        robot.stopper.setPosition(0.95);
        //telemetry.addData("Status", "About to wait 5 sec");
        //telemetry.update();
        sleepTau(1000);
        telemetry.addData("Status", "done");
        telemetry.update();
        int startPos = robot.leftLiftMotor.getCurrentPosition();
        double difference = 5800;
        int targetPosLeft = (int)(robot.leftLiftMotor.getCurrentPosition() - (difference));
        int targetPosRight = (int)(robot.rightLiftMotor.getCurrentPosition() - (difference));

        robot.leftLiftMotor.setTargetPosition(targetPosLeft);
        robot.rightLiftMotor.setTargetPosition(targetPosRight);

        speedLift(1);
        robot.resetTime();
        //getBlockLocation2();
        /*while(targetPosLeft > startPos - 5800){
            while(robot.leftLiftMotor.isBusy()){
                sleepTau(50);
            }
            sleepTau(750);
            targetPosLeft -= difference;
            targetPosRight -= difference;
            robot.leftLiftMotor.setTargetPosition(targetPosLeft);
            robot.rightLiftMotor.setTargetPosition(targetPosRight);
            speedLift(1);
        }*/
        while(opModeIsActive() && robot.getTime() < 10 && robot.leftLiftMotor.isBusy() && robot.rightLiftMotor.isBusy()){
            telemetry.addData("Status", "Dropping robot...");
            telemetry.update();
        }
        speedLift(0);

        getBlockLocation2();
    }

    public void dropLift(){
        robot.stopper.setPosition(0.95);
        //speedLift(0);
        robot.leftLiftMotor.setTargetPosition(robot.rightLiftMotor.getCurrentPosition() + 5850);
        robot.rightLiftMotor.setTargetPosition(robot.leftLiftMotor.getCurrentPosition() + 5850);

        robot.resetTime();
        speedLift(1);

        while(opModeIsActive() && robot.getTime() < 5 && robot.rightLiftMotor.isBusy() && robot.leftLiftMotor.isBusy()){
            telemetry.addData("Status", "Lowering lift...");
            telemetry.update();
        }
        speedLift(0);
        Log.d("lift position", robot.rightLiftMotor.getCurrentPosition() + "");
    }
    //drive forward certain distance at certain speed(speed should be no more than 1), distance is in inches
    public void driveForward(double speed, double distance){
        //speed(speed);
        double startTime = robot.getTime();
        motorPosition = (int)((distance / (6 * Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()- motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()- motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        robot.resetTime();
        speed(speed);
        while(opModeIsActive() && robot.getTime() < 5 && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.backLeftMotor.isBusy()){
            telemetry.addData("Position", robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
        }
        speed(0);
        /*while(opModeIsActive() && robot.frontLeftMotor.isBusy()){
            testDistances();
        }*/
        /*while(opModeIsActive()){
            if(Math.abs(getImuAverageXValue()) >= Math.abs(finalX) && Math.abs(getIMUAverageYValue()) >= Math.abs(finalY)){
                speed(0);
                sleepTau(500);
                break;
            }
        }*/

    }

    private void resetAngle() {
        lastAngle = imu.getAngularOrientation().firstAngle;
        globalAngle = 0;
    }

    private  double getCurrentAngle() {
        double angle = imu.getAngularOrientation().firstAngle;

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;
    }

    public void turnDegrees(double speed, double degree){
        double currentAngle = 0.0;
        initialAngle = getCurrentAngle();
        finalAngle = initialAngle + degree; //assuming turning left is positive degree

        double startTime = robot.getTime();
        currentAngle = initialAngle;
        do{
            double distance = (degree * (2 * robotRotationRadius * Math.PI) / 360);
            motorPosition = (int) ((distance / (6 * Math.PI)) * ticksPerRotation);
            robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + motorPosition);
            robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
            robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + motorPosition);
            robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
            robot.resetTime();
            speed(speed);
            while (opModeIsActive() && robot.getTime() < 3 && robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy() && robot.backRightMotor.isBusy() && robot.backLeftMotor.isBusy()) {
                telemetry.addData("Current Angle::",currentAngle);
                telemetry.update();
            }
            speed(0);

            currentAngle = getCurrentAngle();
            degree = finalAngle - currentAngle;
        } while(Math.abs(currentAngle - finalAngle) > 5 && opModeIsActive() && robot.getTime() < 10);
    }
    /*public void scanMinerals(){
        if(vision.seesSilver()){

        }
    }*/
    public void getBlockLocation(){
        boolean turn = false;
        boolean turn2 = false;
        while (opModeIsActive()) {
            //If time is greater than 15sec, default to center
            if (robot.getTime() > 15) {
                if(turn2){
                    turnDegrees(0.25, 10);
                    sleepTau(750);
                }
                blockLocation = "Center";
                break;
            }
            //Try checking 10 degrees to the left
            if (robot.getTime() > 5 && turn == false) {
                turnDegrees(0.25, 10);
                sleepTau(500);
                turn = true;
                //try checking 10 degrees to the right
            } else if (robot.getTime() > 10 && turn2 == false) {
                turnDegrees(0.25, -20);
                sleepTau(1000);
                turn2 = true;
            }
            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                sleepTau(1500);
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                Log.d("Status", "First call");
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    Log.d("Status", "Objects detected");
                    Log.d("# Objects detected", updatedRecognitions.size() + "");
                    if (updatedRecognitions.size() >= 3 && updatedRecognitions != null) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        int  goldMineralY = Integer.MIN_VALUE;
                        int silverMineral1Y = Integer.MIN_VALUE;
                        int silverMineral2Y = Integer.MIN_VALUE;
                        for (Recognition recognition : updatedRecognitions) {
                            //If it is a gold mineral
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                //If the y value of the top of the gold mineral is greater than the y value of the next lowest
                                //make it the new mineral that we are scanning
                                if(recognition.getTop() > goldMineralY){
                                    goldMineralX = (int) recognition.getLeft();
                                    goldMineralY = (int)recognition.getTop();
                                    Log.d("Width", "" + (Math.abs(recognition.getLeft() - recognition.getRight())));
                                    Log.d("Status", "In more than 3 detected loop" + updatedRecognitions.size());
                                    Log.d("Y of top", recognition.getTop() +"");
                                    Log.d("Gold mineral y", "" + goldMineralY);
                                }
                                //Same for silver, but getting the two lowest instead of just the lowest
                            } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                if(recognition.getTop() > silverMineral1Y && updatedRecognitions.size() > 3) {
                                    silverMineral2X = silverMineral1X;
                                    silverMineral1X = (int) recognition.getLeft();
                                    silverMineral2Y = silverMineral1Y;
                                    silverMineral1Y = (int)recognition.getTop();
                                    Log.d("y of Top Silver 1", recognition.getTop() + "");
                                }else if(recognition.getTop() > silverMineral2Y && updatedRecognitions.size() > 3){
                                    silverMineral2X = (int) recognition.getLeft();
                                    silverMineral2Y = (int) recognition.getTop();
                                }else {
                                    //if it only detects three minerals, just look for the two silvers
                                    if(silverMineral1X == -1){
                                        silverMineral1X = (int) recognition.getLeft();
                                        Log.d("y of Top Silver 1", ""+recognition.getTop());
                                    }else {
                                        silverMineral2X = (int) recognition.getLeft();
                                        Log.d("y of Top Silver 2", "" + recognition.getTop());
                                    }
                                }
                            }
                        }
                        //Find the location of the gold mineral based on the relative location to the silver minerals
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.update();
                                Log.d("Status", "Left");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Left";
                                break;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                telemetry.update();
                                Log.d("Status", "Right");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Right";
                                break;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                telemetry.update();
                                Log.d("Status", "Center");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Center";
                                break;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (robot.tfod != null) {
            //robot.tfod.shutdown();
            //return "tfod null";
        }
        //Turn back to straight based on what turns it has previously made
        if(turn2){
            turnDegrees(0.25, 10);
            sleepTau(750);
        } else if(turn){
            turnDegrees(0.25, -10);
            sleepTau(750);
        }
    }

    public void getBlockLocation2() {
        boolean turn = false;
        boolean turn2 = false;

        blockLocation = "Center"; //default location is center
        robot.resetTime();
        while (opModeIsActive() && robot.getTime() < 4) {
            //Try checking 10 degrees to the left
            if (robot.getTime() > 2 && turn == false) {
                turnDegrees(0.25, 15);
                //sleepTau(500);
                turn = true;
            }

            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                //sleepTau(1500);
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                Log.d("Status", "First call");
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    telemetry.update();
                    Log.d("Status", "Objects detected");
                    Log.d("# Objects detected", updatedRecognitions.size() + "");
                    if (updatedRecognitions.size() >= 2 && updatedRecognitions != null) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        int goldMineralY = -1;
                        int silverMineral1Y = -1;
                        int silverMineral2Y = -1;
                        int mineral1X = -1;
                        int mineral2X = -1;
                        int mineral1Y = -1;
                        int mineral2Y = -1;
                        int mineral1Type = -1; //0 - silver; 1 - gold
                        int mineral2Type = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            //Get the detected 2 minerals with the largest Y values

                            if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) || recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("status", "detected gold or silver");
                                telemetry.update();
                                if (recognition.getTop() > mineral1Y && updatedRecognitions.size() >= 3) {
                                    mineral2X = mineral1X;
                                    mineral2Y = mineral1Y;
                                    mineral2Type = mineral1Type;
                                    mineral1X = (int) recognition.getLeft();
                                    mineral1Y = (int) recognition.getTop();
                                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        silverMineral1X = mineral1X;
                                        silverMineral1Y = mineral1Y;
                                        mineral1Type = 0;
                                    } else {
                                        goldMineralX = mineral1X;
                                        goldMineralY = mineral1Y;
                                        mineral1Type = 1;
                                    }
                                } else if (recognition.getTop() > mineral2Y && updatedRecognitions.size() >= 3) {
                                    mineral2X = (int) recognition.getLeft();
                                    mineral2Y = (int) recognition.getTop();
                                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        silverMineral2X = mineral2X;
                                        silverMineral2Y = mineral2Y;
                                        mineral2Type = 0;
                                    } else {
                                        goldMineralX = mineral2X;
                                        goldMineralY = mineral2Y;
                                        mineral2Type = 1;
                                    }
                                } else if(updatedRecognitions.size() == 2) {
                                    //if it only detects 2 minerals
                                    telemetry.addData("Status", "Two minerals detected");
                                    telemetry.update();
                                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                        if (silverMineral1X == -1) {
                                            silverMineral1X = (int) recognition.getLeft();
                                            silverMineral1Y = (int) recognition.getTop();
                                            mineral1Type = 0;
                                        } else if (silverMineral2X == -1) {
                                            silverMineral2X = (int) recognition.getLeft();
                                            silverMineral2Y = (int) recognition.getTop();
                                            mineral2Type = 0;
                                        }
                                    } else {
                                        if (goldMineralX == -1) {
                                            goldMineralX = (int) recognition.getLeft();
                                            goldMineralY = (int) recognition.getTop();
                                            if(mineral1Type == -1)
                                                mineral1Type = 1;
                                            else
                                                mineral2Type = 1;
                                        }
                                    }
                                }
                            }
                        }
                        telemetry.addData("Mineral 1 type" + mineral1Type, "Mineral 2 type" + mineral2Type);
                        telemetry.update();
                        //Find the location of the gold mineral based on the relative location to the silver minerals
                        if (mineral1Type == 0 && mineral2Type == 0) {
                            //gold mineral not detected among the left 2 positions, so it is at right
                            telemetry.addData("Gold Mineral Position", "Right");
                            telemetry.update();
                            Log.d("Status", "Right");
                            Log.d("Status", goldMineralX + " " + goldMineralY);
                            blockLocation = "Right";
                            break;
                        } else if (mineral1Type == 0 && mineral2Type == 1) {
                            if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.update();
                                Log.d("Status", "Left");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Left";
                                break;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                telemetry.update();
                                Log.d("Status", "Center");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Center";
                                break;
                            }
                        }
                        else if (mineral1Type == 1 && mineral2Type == 0) {
                            if (goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.update();
                                Log.d("Status", "Left");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Left";
                                break;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                telemetry.update();
                                Log.d("Status", "Center");
                                Log.d("Status", goldMineralX + " " + goldMineralY);
                                blockLocation = "Center";
                                break;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        //Turn back to straight based on what turns it has previously made
        if (turn) {
            turnDegrees(0.25, -15);
            //sleepTau(750);
        }
    }

    public void knockBlockOff(String block){
        driveForward(0.25, Math.sqrt(2) * 12);
        sleepTau(1000);
        if(block.equals("Left") || block.equals("Right")){
            turnDegrees(0.25, block.equals("Left") ? 50: -40);
            sleepTau(1500);
            driveForward(0.25, 25);
            sleepTau(3500);
            turnDegrees(0.25, block.equals("Left") ? -80: 80);
            sleepTau(1000);
            if(block.equals("Left")){
                driveForward(0.25, 24);
                sleepTau(1250);
                turnDegrees(0.25, 80);
                sleepTau(1000);
                dropArm();
                turnDegrees(0.25, 87);
                sleepTau(1000);
                driveForward(0.25, Math.sqrt(2) * 9 + 1);
                sleepTau(2000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                dropLift();
                //calls arm method
                //dropLift();
                //driveForwardToCrater();
            }else{
                driveForward(0.25, 31);
                sleepTau(3500);
                dropArm();
                turnDegrees(0.25, 90);
                sleepTau(1500);
                //driveForward(0.5, Math.sqrt(2) * 12);
                //sleepTau(2000);
                //turnDegrees(.5, 40);
                //sleepTau(1000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                dropLift();
                //  driveForwardToCrater();
                //turnDegrees(0.5,  );
                // sleepTau(2000);
            }

        } else if(block.equals("Center")){
            telemetry.addData("Block position", "Center");
            telemetry.update();
            driveForward(0.25, 2 * Math.sqrt(2) * 12);
            sleepTau(1500);
            turnDegrees(0.25, 90);
            dropArm();
            driveForward(0.25, 5);
            sleepTau(1000);
            turnDegrees(0.25, 10);
            sleepTau(1000);
            driveForward(0.25, 16);
            sleepTau(1000);
            turnDegrees(0.25, 30);
            sleepTau(1000);
            dropLift();
            //driveForwardToCrater();
        }
    }

    /*public void driveForwardToCrater(){
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);
        robot.frontLeftMotor.setPower(-0.75);
        robot.backLeftMotor.setPower(-0.75);
        /*while(opModeIsActive()){
            if(imu.getAngularOrientation().thirdAngle - thirdAngleZero >= 352){
                sleepTau(500);
                speed(0);
                sleepTau(500);
                break;
            }
        }*/

    //}

    /*public void testAndBackUpIntoCrater(){
        double initialDist = robot.ultrasonicSensor.getDistance(DistanceUnit.INCH);
        stopRobot();
        sleepTau(500);
        if(robot.ultrasonicSensor.getDistance(DistanceUnit.INCH) <= initialDist - 1 && initialDist <= 4){
            driveForward(0.5, -6);
            sleepTau(750);
            turnDegrees(0.5, 180);
            sleepTau(750);
            driveForwardToCrater();
            driveForward(0.5, 12);
            sleepTau(3000);
        }
    }*/

    public void stopRobot() {
        speed(0);
    }

    public void driveForwardAndDropLift(double distance){
        speed(0.5);
        motorPosition = (int)((distance / (6 * Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()- motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()- motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        dropLift();
        while(robot.frontLeftMotor.isBusy()){
            sleepTau(50);
        }
        speed(0);
    }

    public void sleepTau(long milliSec){
        double start = robot.getTime();
        double end = start + milliSec/1000.0;
        while(opModeIsActive()){
            if(robot.getTime() >= end){
                break;
            }
            telemetry.addData("Status:",  "Alive");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {}

    /*public void testDistances(){
        while(opModeIsActive()) {
            if (getDistance() < 6) {
                speed(0);
                break;
            }
        }
    }*/
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

