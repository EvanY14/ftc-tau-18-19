package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public String blockLocation = "Center";
    private double initialX;
    private double initialY;
    ElapsedTime period = new ElapsedTime();
    private final double stopper_up = 0.95;
    private final double stopper_down = 1;

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

    public BNO055IMU imu;
    private BNO055IMU imu1;
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
    private double zeroX;
    private double zeroY;
    private double zeroZ;
    private double zeroHeading;
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
        robot.init_auto_IMU(hwMap, telemetry);
        boolean useFullRes = true;
        Context context = hardwareMap.appContext;
        //cameraManager.initialize(context, useFullRes, this);
        //imageProcessor.initialize(useFullRes, this, true, cameraManager.height, cameraManager.width);
        //robot.imageTrackables.activate();

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        parameters.loggingTag = "IMU1";
        imu1 = hardwareMap.get(BNO055IMU.class, "imu1");
        imu1.initialize(parameters);

        // Set up our telemetry dashboard
        //composeTelemetry(imu, imu1, telemetry);

        // Start the logging of measured acceleration

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("imu1 calib status", imu1.getCalibrationStatus().toString());
        telemetry.update();
        zeroHeading = imu.getAngularOrientation().firstAngle;
        zeroX = imu.getPosition().x;
        zeroY = imu.getPosition().y;
        zeroZ = imu.getPosition().z;
        telemetry.addData("Readiness", "Press Play to start");
        telemetry.update();



        // Wait until we're told to go
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        imu1.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        robot.tfod.activate();
        period.startTime();
    }
    //Behind the scenes methods

    //Sets speeds of motors
    private void speed(double speed){
        robot.frontLeftMotor.setPower(-speed);
        robot.frontRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(-speed);
        robot.backRightMotor.setPower(speed);
    }

    public void speedLift(double speed){
        robot.rightLiftMotor.setPower(speed);
        //robot.leftLiftMotor.setPower(speed);
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
        sleepTau(500);
        telemetry.addData("Status", "done");
        telemetry.update();
        speedLift(1);
        //robot.leftLiftMotor.setTargetPosition((int)robot.leftLiftMotor.getCurrentPosition() + 5700);
        robot.rightLiftMotor.setTargetPosition((int)robot.rightLiftMotor.getCurrentPosition() + 5700);
        blockLocation = getBlockLocation();
    }

    public void dropLift(){
        robot.stopper.setPosition(0.95);
        speedLift(1);
        //robot.leftLiftMotor.setTargetPosition(robot.rightLiftMotor.getCurrentPosition() - 5600);
        //robot.rightLiftMotor.setTargetPosition(robot.leftLiftMotor.getCurrentPosition() - 5600);
        sleepTau(3000);
    }
    //drive forward certain distance at certain speed(speed should be no more than 1), distance is in inches
    public void driveForward(double speed, double distance){

        initialX = imu.getPosition().x / mmPerInch;
        initialY = imu.getPosition().y / mmPerInch;
        double heading = imu.getAngularOrientation().firstAngle;
        double deltaX = Math.toDegrees(Math.cos(heading)) * distance;
        double deltaY = Math.toDegrees(Math.sin(heading)) * distance;
        finalX = initialX + deltaX;
        finalY = initialY + deltaY;
        speed(speed);
        while(opModeIsActive()){
            updateTelemetry(telemetry);
            if(Math.abs(imu.getPosition().x) >= Math.abs(finalX) && Math.abs(imu.getPosition().y) >= Math.abs(finalY)){
                speed(0);
                sleepTau(500);
                break;
            }
        }

    }

    public void turnDegrees(double speed, double degree){
        speed(speed);
        initialAngle = imu.getAngularOrientation().firstAngle;
        finalAngle = initialAngle + degree;
        double distance = (degree * (2 * robotRotationRadius * Math.PI) / 360);
        motorPosition = (int)((distance / (6*Math.PI)) * ticksPerRotation);
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition()+ motorPosition);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + motorPosition);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition()+ motorPosition);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + motorPosition);
        while(opModeIsActive()){
            if(Math.abs(imu.getAngularOrientation().firstAngle) >= Math.abs(finalAngle)){
                speed(0);
                sleepTau(500);
                break;
            }
        }
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
        boolean turn = false;
        boolean turn2 = false;
            while (opModeIsActive()) {
                if (robot.getTime() > 15) {
                    if(turn2){
                        turnDegrees(0.25, 10);
                        sleepTau(750);
                    }
                    return "Center";
                }
                if (robot.getTime() > 5 && turn == false) {
                    turnDegrees(0.25, 10);
                    sleepTau(500);
                    turn = true;
                } else if (robot.getTime() > 10 && turn2 == false) {
                    turnDegrees(0.25, -20);
                    sleepTau(1000);
                    turn2 = true;
                }
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
        if(turn2){
                turnDegrees(0.25, 10);
                sleepTau(750);
        } else if(turn){
                turnDegrees(0.25, -10);
                sleepTau(750);
        }
        return location;
    }

    public void knockBlockOff(String block){
        driveForward(0.25, Math.sqrt(2) * 12);
        sleepTau(1000);
        if(block.equals("Left") || block.equals("Right")){
            turnDegrees(0.25, block.equals("Left") ? 45: -45);
            sleepTau(1250);
            driveForward(0.25, 25);
            sleepTau(3500);
            turnDegrees(0.25, block.equals("Left") ? -90: 90);
            sleepTau(1000);
            if(block.equals("Left")){
                driveForward(0.25, 24);
                sleepTau(1250);
                turnDegrees(0.25, 90);
                sleepTau(1000);
                dropArm();
                turnDegrees(0.25, 75);
                sleepTau(750);
                driveForward(0.25, Math.sqrt(2) * 9 + 1);
                sleepTau(2000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                //calls arm method
                //dropLift();
                driveForward(1, 60);
                sleepTau(5000);
            }else{
                driveForward(0.25, 31);
                sleepTau(3500);
                dropArm();
                turnDegrees(0.5, 80);
                sleepTau(750);
                //driveForward(0.5, Math.sqrt(2) * 12);
                //sleepTau(2000);
                //turnDegrees(.5, 40);
                //sleepTau(1000);
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                //calls arm method
                //dropLift();
                driveForward(0.5, 60);
                sleepTau(5000);
                //turnDegrees(0.5,  );
               // sleepTau(2000);
            }

        } else if(block.equals("Center")){
            telemetry.addData("Block position", "Center");
            telemetry.update();
            driveForward(0.25, 2 * Math.sqrt(2) * 12);
            sleepTau(1500);
            turnDegrees(0.5, 90);
            sleepTau(750);
            dropArm();
            turnDegrees(0.5, 26.5);
            sleepTau(750);
            driveForward(0.25, 20);
            sleepTau(1000);
            turnDegrees(0.25, 20);
            sleepTau(750);
            driveForward(0.5, 55);
            sleepTau(3000);
        }
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
    public void composeTelemetry(final BNO055IMU imu, final BNO055IMU imu1, Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
            angles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity1 = imu1.getGravity();
            position = imu.getPosition();
            position1 = imu1.getPosition();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString() + " " + imu1.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString() + " " + imu1.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle-zeroHeading) + " " + formatAngle(angles1.angleUnit, angles.firstAngle-zeroHeading);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle) + " " + formatAngle(angles1.angleUnit, angles1.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle) + " " + formatAngle(angles1.angleUnit, angles1.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString() + " " + gravity1.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel)) + " " + String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity1.xAccel*gravity1.xAccel
                                        + gravity1.yAccel*gravity1.yAccel
                                        + gravity1.zAccel*gravity1.zAccel));
                    }
                });
        telemetry.addLine()
                .addData("position", new Func<String>(){
                    @Override public String value(){
                        return "" + String.format(Locale.getDefault(), "%.3f", position.x - zeroX) +" "+ String.format(Locale.getDefault(), "%.3f", position.y - zeroY);
                    }
                });

        telemetry.update();
        }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getImuAverageXValue(){
        updateTelemetry(telemetry);
        return ((imu.getPosition().x + imu1.getPosition().x)/2.0);
    }
    public double getIMUAverageYValue(){
        updateTelemetry(telemetry);
        return((imu.getPosition().y + imu1.getPosition().y)/2.0);
    }
    public double getIMUAverageZValue(){
        updateTelemetry(telemetry);
        return (imu.getPosition().z + imu1.getPosition().z) / 2.0;
    }
    public double getImuAverageRotation(){
        updateTelemetry(telemetry);
        return ((imu.getAngularOrientation().firstAngle + imu1.getAngularOrientation().firstAngle) / 2.0);
    }
}