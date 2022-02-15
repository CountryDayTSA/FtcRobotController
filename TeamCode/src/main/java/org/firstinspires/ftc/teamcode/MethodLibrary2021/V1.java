package org.firstinspires.ftc.teamcode.MethodLibrary2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RealSense;

@Autonomous
public class V1 extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    DcMotor frontrightmotor;
    DcMotor frontleftmotor;
    DcMotor backrightmotor;
    DcMotor backleftmotor;
    DcMotor baseMotor;
    DcMotor topMotor;
    Servo boxServo;
    Servo trapDoor;
    DcMotor spinMotor;
    double trapDoorPosition = 0.35;
    double speedScale = 0.5;
    double frontrightmotorpower;
    double frontleftmotorpower;
    double backrightmotorpower;
    double backleftmotorpower;
    double boxServoPosition = 0.32;
    double spinMotorPower = 0;

    int basePosition=0;

    double NEW_P = 7.0;
    double NEW_I = 7;
    double NEW_D = 0.2;
    double NEW_F = 8;
    // 6 3.5 .2 5
    BNO055IMU imu;
    Orientation angles;

    final double countsPerInch = 42;

    //RealSense DepthSensor = null;
    //RealSense camera = new RealSense(hardwareMap.appContext);

    public void runOpMode() {
        hardwaremap();
        waitForStart();
        while (opModeIsActive()) {
            move(90, 4);

        }
    }

    public void hardwaremap() {
        frontrightmotor = hardwareMap.dcMotor.get("FRM");
        frontleftmotor = hardwareMap.dcMotor.get("FLM");
        backrightmotor = hardwareMap.dcMotor.get("BRM");
        backleftmotor = hardwareMap.dcMotor.get("BLM");
        baseMotor = hardwareMap.dcMotor.get("base");
        topMotor = hardwareMap.dcMotor.get("top");
        boxServo = hardwareMap.servo.get("box");
        trapDoor = hardwareMap.servo.get("door");
        spinMotor = hardwareMap.dcMotor.get("spinner");

        DcMotorControllerEx baseMotorController = (DcMotorControllerEx)baseMotor.getController();
        PIDFCoefficients pidf = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotorController.setPIDFCoefficients((baseMotor).getPortNumber(), DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public boolean isMoving() {
        return frontrightmotor.isBusy() || frontleftmotor.isBusy() || backrightmotor.isBusy() || backleftmotor.isBusy();
    }

    public void move(int angle, double seconds) {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double initHeading = AngleUnit.DEGREES.normalize(angles.firstAngle);
        double radHeading = Math.toRadians(initHeading) + Math.PI/2;
        if (radHeading<0) radHeading+=2*Math.PI;
        telemetry.addLine("H: " + Math.toDegrees(radHeading));
        double radAngle = Math.toRadians(angle) + Math.PI/2;
        if (radAngle<0) radAngle+=Math.PI*2;
        telemetry.addLine("I: " + Math.toDegrees(radAngle));
        telemetry.update();
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
        telemetry.update();
    }

}
