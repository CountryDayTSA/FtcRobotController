package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

    public void init() {
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

    public void loop() {
        double turn = gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        double heading = AngleUnit.DEGREES.normalize(angles.firstAngle);

        double speed = Math.sqrt(x*x+y*y);
        if (Math.abs(speed)>1) speed/=Math.abs(speed);

        double radJoystick = Math.atan2(y,x);
        if (radJoystick<0) radJoystick+=(2*Math.PI);
        double radHeading = heading/(180/Math.PI)+(Math.PI/2);
        if (radHeading<0) radHeading+=(2*Math.PI);

        double targetHeading = radJoystick+(Math.PI/2-radHeading);

        frontrightmotorpower = (Math.sin(targetHeading)-Math.cos(targetHeading))/Math.sqrt(2);
        frontleftmotorpower = (Math.sin(targetHeading)+Math.cos(targetHeading))/Math.sqrt(2);
        backrightmotorpower = (Math.sin(targetHeading)+Math.cos(targetHeading))/Math.sqrt(2);
        backleftmotorpower = (Math.sin(targetHeading)-Math.cos(targetHeading))/Math.sqrt(2);

        if (gamepad1.a) boxServoPosition=0.8;
        else if (gamepad1.b) boxServoPosition=0.5;
        else if (gamepad1.y) boxServoPosition=0.32;

        if (gamepad1.x) {
            if (spinMotorPower==0) spinMotorPower=1;
            else spinMotorPower=0;
        }

        if (gamepad1.right_bumper) trapDoorPosition=.35;
        else if (gamepad1.left_bumper) {
            trapDoorPosition=.6;
        }

        if (gamepad1.dpad_up) basePosition+=2;
        else if (gamepad1.dpad_down) basePosition-=2;

        baseMotor.setTargetPosition(basePosition);
        baseMotor.setPower(1);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //baseMotor.setPower(gamepad1.left_trigger);

        spinMotor.setPower(spinMotorPower);

        frontrightmotor.setPower((0.7*(speed*frontrightmotorpower)-.3*turn)*speedScale);
        frontleftmotor.setPower((0.7*(speed*frontleftmotorpower)+.3*turn)*speedScale);
        backrightmotor.setPower((0.7*(speed*backrightmotorpower)-.3*turn)*speedScale);
        backleftmotor.setPower((0.7*(speed*backleftmotorpower)+.3*turn)*speedScale);

        topMotor.setPower(gamepad1.left_trigger-0.2*gamepad1.right_trigger);
        boxServo.setPosition(boxServoPosition);
        trapDoor.setPosition(trapDoorPosition);

        telemetry.addLine("Box: " + boxServoPosition);
        telemetry.addLine("Base pos: " + basePosition);
        telemetry.addLine("Base act: " + baseMotor.getCurrentPosition());
        telemetry.addLine("Top: " + topMotor.getCurrentPosition());
        telemetry.addLine("FRM: " + frontrightmotor.getCurrentPosition());
        telemetry.addLine("FLM: " + frontleftmotor.getCurrentPosition());
        telemetry.addLine("BRM: " + backrightmotor.getCurrentPosition());
        telemetry.addLine("BLM: " + backleftmotor.getCurrentPosition());

        telemetry.addLine("DOOR: " + trapDoorPosition);
        telemetry.addLine("Gamepad1.y: " + y);
        telemetry.addLine("Gamepad1.x: " + x);
        telemetry.addLine("Gamepad2.x: " + turn);
        telemetry.addLine("IMU: " + heading);
        telemetry.addLine("Joystick: " + radJoystick*(180/Math.PI));
        telemetry.addLine("Current heading: " + radHeading*(180/Math.PI));
        telemetry.addLine("Target Heading: " + targetHeading);
        telemetry.update();
    }
}
