package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    double speedScale = 0.5;
    double frontrightmotorpower;
    double frontleftmotorpower;
    double backrightmotorpower;
    double backleftmotorpower;

    BNO055IMU imu;
    Orientation angles;

    public void init() {
        frontrightmotor = hardwareMap.dcMotor.get("FRM");
        frontleftmotor = hardwareMap.dcMotor.get("FLM");
        backrightmotor = hardwareMap.dcMotor.get("BRM");
        backleftmotor = hardwareMap.dcMotor.get("BLM");
        frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        double radJoystick = Math.atan2(y,x);
        double radHeading = heading/(180/Math.PI)+(Math.PI/2);
        if (radHeading<0) radHeading+=(2*Math.PI);
        if (radJoystick<0) radJoystick+=(2*Math.PI);

        frontrightmotor.setPower(frontrightmotorpower*speedScale);
        frontleftmotor.setPower(frontleftmotorpower*speedScale);
        backrightmotor.setPower(backrightmotorpower*speedScale);
        backleftmotor.setPower(backleftmotorpower*speedScale);

        telemetry.addLine("Gamepad1.y: " + y);
        telemetry.addLine("Gamepad1.x: " + x);
        telemetry.addLine("IMU: " + heading);
        telemetry.addLine("Joystick: " + radJoystick*(180/Math.PI));
        telemetry.addLine("Current heading: " + radHeading*(180/Math.PI));

    }
}
