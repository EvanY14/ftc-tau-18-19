package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

/**
 * Created by Evan Yu on 9/16/2018.
 */

public class Hardware extends OpMode {
    //Drive Motors******************
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    //******************************

    //Lift Motors*******************
    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;
    //******************************

    //Servos************************
    public Servo stopper = null;
    public Servo markerArm = null;
    //******************************

    //Sensors***************************
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
   // public ModernRoboticsI2cRangeSensor ultrasonicSensor = null;
    //******************************

    //Vision************************
    OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackable imageTemplate;
    public VuforiaTrackables imageTrackables;
    private String VUFORIA_KEY = "AUTPgLj/////AAABmftxO0IFGU3urmaLhFDDt+04jQVVUEnMoybqfXkW+2kDybcXkSk00wQ1RARTA6i+W3x8pWjVDY/xcKrLUwZZKYSdeSlSWW+nMK4s5AEaTS8K0Re8OrF3JF3zmHz4julP101iBl7+dpVOEFw10laj2E0q0bvw9vqvXMMjg8J3zdXiDS4zzHPRl0Iwx6iaH4ZmmE4VqXiJ8kXrZ9bc897oR4FcC01mF+cX3x6oi5e8ZpQanSDPp2/IBbvUxi/oe2ImrNpZTczvZLMwYMTQqgfeN9Ewz5KtCbAwfCLARiW5QZ/EOOdlLfGIPXGYesLuVPswhWP5HCCCrberCUZ+y+2OGj7+SlesgFSD8qwWNMQh+Erx";
    public TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //******************************

    //Other*************************
    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();
    public double leftLiftBottom = 0;
    public double rightLiftBottom = 0;
    public double leftLiftTop = 0;
    public double rightLiftTop = 0;
    //******************************

    public Hardware(){
        hwMap = null;
    }

    @Override
    public void init() { }

    @Override
    public void loop() { }

    public void init(HardwareMap hwMap, Telemetry telemetry){

        this.hwMap = hwMap;
        //init drive motors
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("back_left");
        backRightMotor = hwMap.dcMotor.get("back_right");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //init servos
        stopper = hwMap.servo.get("lift_stopper");
        markerArm = hwMap.servo.get("marker_arm");

        markerArm.setPosition(0);
        stopper.setPosition(1);

        //init lift motors
        leftLiftMotor = hwMap.dcMotor.get("left_lift");
        rightLiftMotor = hwMap.dcMotor.get("right_lift");

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);

        //initialize variables
        leftLiftBottom = leftLiftMotor.getCurrentPosition();
        rightLiftBottom = rightLiftMotor.getCurrentPosition();
        leftLiftTop = leftLiftBottom + 6000;
        rightLiftTop = rightLiftBottom + 6000;
    }

    public void init_auto(HardwareMap hwMap, Telemetry telemetry){
        init(hwMap, telemetry);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sensors
       // ultrasonicSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");
        /*******/
        //Vision stuff
        /*int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Vuparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        Vuparameters.vuforiaLicenseKey = "AUTPgLj/////AAABmftxO0IFGU3urmaLhFDDt+04jQVVUEnMoybqfXkW+2kDybcXkSk00wQ1RARTA6i+W3x8pWjVDY/xcKrLUwZZKYSdeSlSWW+nMK4s5AEaTS8K0Re8OrF3JF3zmHz4julP101iBl7+dpVOEFw10laj2E0q0bvw9vqvXMMjg8J3zdXiDS4zzHPRl0Iwx6iaH4ZmmE4VqXiJ8kXrZ9bc897oR4FcC01mF+cX3x6oi5e8ZpQanSDPp2/IBbvUxi/oe2ImrNpZTczvZLMwYMTQqgfeN9Ewz5KtCbAwfCLARiW5QZ/EOOdlLfGIPXGYesLuVPswhWP5HCCCrberCUZ+y+2OGj7+SlesgFSD8qwWNMQh+Erx";
        Vuparameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(Vuparameters);*/
        //imageTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        //imageTemplate = imageTrackables.get(0);
        //imageTemplate.setName("roverVuMarkTemplate");
        //imageTrackables.activate();
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hwMap);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);


    }
    public void init_auto_IMU(HardwareMap hwMap, Telemetry telemetry){
        init_auto(hwMap, telemetry);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getTime(){
        return period.time();
    }

    public void sleepTau(long millis) throws InterruptedException {
        period.wait(millis);
        telemetry.addData("Status", "Waited" + millis/1000.0 + "seconds");
        telemetry.update();
    }

    public void initVuforia(HardwareMap hwMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
       this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    public void initTfod(HardwareMap hwMap) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void resetTime(){
        period.reset();
    }
    public void startTime(){
        period.startTime();
    }
}