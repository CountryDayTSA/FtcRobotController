package org.firstinspires.ftc.teamcode.MethodLibrary2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RealSense;

public class V1 extends LinearOpMode {
    public void runOpMode() {
        while (getRuntime()<30 && opModeIsActive()) {

        }
    }
    ElapsedTime timer = new ElapsedTime();
    DcMotor frontrightmotor;
    DcMotor frontleftmotor;
    DcMotor backrightmotor;
    DcMotor backleftmotor;
    BNO055IMU imu;
    RealSense DepthSensor = null;
    RealSense camera = new RealSense(hardwareMap.appContext);

    final double countsPerInch = 40;

    public void hardwaremap() {
        frontrightmotor = hardwareMap.dcMotor.get("FRM");
        frontleftmotor = hardwareMap.dcMotor.get("FLM");
        backrightmotor = hardwareMap.dcMotor.get("BRM");
        backleftmotor = hardwareMap.dcMotor.get("BLM");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void reset() {
        timer.reset();
    }

    public double seconds() {
        return timer.seconds();
    }

    public void encoderTelemetry() {
        telemetry.addLine("FRM: " + frontrightmotor.getCurrentPosition());
        telemetry.addLine("FLM: " + frontleftmotor.getCurrentPosition());
        telemetry.addLine("BRM: " + backrightmotor.getCurrentPosition());
        telemetry.addLine("BLM: " + backleftmotor.getCurrentPosition());
    }

}
