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
    double speedScale = 1;
    double frontrightmotorpower;
    double frontleftmotorpower;
    double backrightmotorpower;
    double backleftmotorpower;
    double boxServoPosition = 0;
    double boxServoCorrection = 0;
    double spinMotorPower = 0;
    double speed;

    int basePosition=0;

    byte spin=0;

    byte scale=0;


    double NEW_P = 9;
    double NEW_I = 2;
    double NEW_D = 0.5;
    double NEW_F = 0.0;

    BNO055IMU imu;
    Orientation angles;

    double pi = Math.PI;

    final double countsPerInch = 42;

    //RealSense DepthSensor = null;
    //RealSense camera = new RealSense(hardwareMap.appContext);

    public void runOpMode() {
        hardwaremap();
        boxServo.setPosition(0.1);
        trapDoor.setPosition(0.35);
        waitForStart();

        move(243, 28.5, 0.6);
        move(0, 2.4, 0.3);
        while (getRuntime()<8) {
            spinMotor.setPower(0.4);
            baseMotor.setTargetPosition(390);
            baseMotor.setPower(1);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            boxServo.setPosition(0.7);
        }
        move(117, 43, 0.6);
        while (getRuntime()<14) {
            trapDoor.setPosition(0.7);
        }
        trapDoor.setPosition(0.35);
        boxServo.setPosition(0.1);
        while (getRuntime()<18) {
            move(270, 7, 0.5);
            baseMotor.setTargetPosition(200);
            baseMotor.setPower(1);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while (getRuntime()<25) {
            baseMotor.setTargetPosition(25);
            baseMotor.setPower(1);
            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            move(50, 45, 0.7);
        }
        move(90, 75, 0.9);

    }

    public void resetGyro() {
        BNO055IMU imu;

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
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

    public void move(int angle, double inches, double power) {
        encoderReset();
        resetGyro();
        double T = countsPerInch*inches;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double initHeading = AngleUnit.RADIANS.normalize(angles.firstAngle)+pi/2;
        if (initHeading<0) initHeading+=2*pi;
        telemetry.addLine("Init: " + Math.toDegrees(initHeading));

        double targetHeading = Math.toRadians(angle);
        telemetry.addLine("Param: " + Math.toDegrees(targetHeading));
        double H = (pi/2-initHeading)+targetHeading;
        telemetry.addLine("H: " + Math.toDegrees(H));

        double target1 = Math.sin(H)-Math.cos(H);
        double target2 = Math.sin(H)+Math.cos(H);

        frontrightmotor.setTargetPosition((int) (target1*T));
        telemetry.addLine("FRM: " + frontrightmotor.getTargetPosition());
        frontleftmotor.setTargetPosition((int) (target2*T));
        telemetry.addLine("FLM: " + frontleftmotor.getTargetPosition());
        backrightmotor.setTargetPosition((int) (target2*T));
        backleftmotor.setTargetPosition((int) (target1*T));
        telemetry.update();

        runAll();

        while(isMoving()) {
            frontrightmotor.setPower((target1/Math.sqrt(2))*power);
            frontleftmotor.setPower((target2/Math.sqrt(2))*power);
            backrightmotor.setPower((target2/Math.sqrt(2))*power);
            backleftmotor.setPower((target1/Math.sqrt(2))*power);
        }
        brake();
    }

    public void turn(int degrees, String direction, double power) {
        encoderReset();
        if (direction.equals("right")) {
            frontrightmotor.setTargetPosition(degrees*-9);
            frontleftmotor.setTargetPosition(degrees*9);
            backrightmotor.setTargetPosition(degrees*-9);
            backleftmotor.setTargetPosition(degrees*9);
            runAll();
            while (isMoving()) {
                frontrightmotor.setPower(-power);
                frontleftmotor.setPower(power);
                backrightmotor.setPower(-power);
                backleftmotor.setPower(power);
            }
            brake();
        }
        else {
            frontrightmotor.setTargetPosition(degrees*9);
            frontleftmotor.setTargetPosition(degrees*-9);
            backrightmotor.setTargetPosition(degrees*9);
            backleftmotor.setTargetPosition(degrees*-9);
            runAll();
            while (isMoving()) {
                frontrightmotor.setPower(power);
                frontleftmotor.setPower(-power);
                backrightmotor.setPower(power);
                backleftmotor.setPower(-power);
            }
            brake();
        }
    }

    public void encoderReset() {
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runAll() {
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void brake() {
        frontrightmotor.setPower(0);
        frontleftmotor.setPower(0);
        backrightmotor.setPower(0);
        backleftmotor.setPower(0);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
