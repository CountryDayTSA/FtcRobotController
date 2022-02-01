package org.firstinspires.ftc.teamcode.MethodLibrary2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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

    //final double wheelRadius = 4.8; //cm
    //final double countsPerInch = (2*wheelRadius*Math.PI)/2.54;
    RealSense DepthSensor = null;
    RealSense camera = new RealSense(hardwareMap.appContext);

    public void runOpMode() {
        while (getRuntime()<30 && opModeIsActive()) {

        }
    }

    final double countsPerInch = 40;

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

        frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void move(int angle, double inches) {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = AngleUnit.DEGREES.normalize(angles.firstAngle);

        double radHeading = heading/(180/Math.PI)+(Math.PI/2);
        if (radHeading<0) radHeading+=(2*Math.PI);

        double target1 = Math.sin(angle)-Math.cos(angle);
        double target2 = Math.sin(angle)+Math.cos(angle);
        target1*=inches*40;
        target2*=inches*40;
        int target11 = (int) target1;
        int target22 = (int) target2;
        frontrightmotor.setTargetPosition(target11);
        frontleftmotor.setTargetPosition((target22));
        backrightmotor.setTargetPosition(target22);
        backleftmotor.setTargetPosition(target11);

        while (isMoving()) {
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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
    }

}
