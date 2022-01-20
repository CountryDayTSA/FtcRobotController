package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class MainRun extends OpMode {
    DcMotor frontrightmotor;
    DcMotor frontleftmotor;
    DcMotor backrightmotor;
    DcMotor backleftmotor;
    DcMotor baseMotor;
    DcMotor topMotor;
    Servo boxServo;
    double speedScale = 0.5;
    double frontrightmotorpower;
    double frontleftmotorpower;
    double backrightmotorpower;
    double backleftmotorpower;
    double servoposition = 0;

    int basePosition=0;

    BNO055IMU imu;
    Orientation angles;

    final double wheelRadius = 4.8; //cm
    final double countsPerInch = (2*wheelRadius*Math.PI)/2.54;

    public void init() {
        frontrightmotor = hardwareMap.dcMotor.get("FRM");
        frontleftmotor = hardwareMap.dcMotor.get("FLM");
        backrightmotor = hardwareMap.dcMotor.get("BRM");
        backleftmotor = hardwareMap.dcMotor.get("BLM");
        baseMotor = hardwareMap.dcMotor.get("base");
        topMotor = hardwareMap.dcMotor.get("ratio");
        boxServo = hardwareMap.servo.get("box");

        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void loop() {
        double turn = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = AngleUnit.DEGREES.normalize(angles.firstAngle);

        frontrightmotorpower = y-turn-x;
        frontleftmotorpower = y+turn+x;
        backrightmotorpower = y-turn+x;
        backleftmotorpower = y+turn-x;

        if (gamepad1.a) servoposition=0.8;
        else if (gamepad1.b) servoposition=0.41;
        else if (gamepad1.y) servoposition=0.1;

        if (gamepad1.dpad_up && baseMotor.getCurrentPosition()<130) basePosition+=5;
        else if (gamepad1.dpad_down && baseMotor.getCurrentPosition()>5) basePosition-=5;

        double radJoystick = Math.atan2(y,x);
        if (radJoystick<0) radJoystick+=(2*Math.PI);
        double radHeading = heading/(180/Math.PI)+(Math.PI/2);
        if (radHeading<0) radHeading+=(2*Math.PI);

        frontrightmotor.setPower(frontrightmotorpower*speedScale);
        frontleftmotor.setPower(frontleftmotorpower*speedScale);
        backrightmotor.setPower(backrightmotorpower*speedScale);
        backleftmotor.setPower(backleftmotorpower*speedScale);
        //baseMotor.setPower(gamepad2.left_stick_y*.3);
        baseMotor.setTargetPosition(basePosition);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topMotor.setPower(gamepad2.right_stick_y*.3);
        boxServo.setPosition(servoposition);

        telemetry.addLine("Box: " + servoposition);
        telemetry.addLine("Base: " + baseMotor.getCurrentPosition());
        telemetry.addLine("Top: " + topMotor.getCurrentPosition());
        telemetry.addLine("FRM: " + frontrightmotor.getCurrentPosition());
        telemetry.addLine("FLM: " + frontleftmotor.getCurrentPosition());
        telemetry.addLine("BRM: " + backrightmotor.getCurrentPosition());
        telemetry.addLine("BLM: " + backleftmotor.getCurrentPosition());

        telemetry.addLine("Gamepad1.y: " + y);
        telemetry.addLine("Gamepad1.x: " + x);
        telemetry.addLine("IMU: " + heading);
        telemetry.addLine("Joystick: " + radJoystick*(180/Math.PI));
        telemetry.addLine("Current heading: " + radHeading*(180/Math.PI));
        telemetry.update();
    }
}
