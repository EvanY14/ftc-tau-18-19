package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "Imu test", group = "Tau")
public class IMU_Test extends LinearOpMode {
    Hardware robot = new Hardware();
    public BNO055IMU imu;
    ArrayList<Double> accelerationX = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
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
        /** accelerometer power mode. See Section 3.5.2 (p27) and Section 4.2.2 (p77) of the BNO055 specification
         parameters.accelPowerMode      = BNO055IMU.AccelPowerMode.NORMAL;

         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         // and named "imu".*/
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("status", "Press play to start");
        telemetry.update();
        waitForStart();
        while(robot.getTime() < 5){
            accelerationX.add(imu.getAcceleration().xAccel);
        }
        Log.d("Accel size",accelerationX.size() + "");
    }
}
