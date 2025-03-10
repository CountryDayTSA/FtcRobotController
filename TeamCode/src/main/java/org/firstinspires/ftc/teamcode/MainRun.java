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
import java.util.Arrays;

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

        //frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        if (Math.abs(y)>=Math.abs(x)) speed=Math.abs(y);
        else speed=Math.abs(x);

        double radJoystick = Math.atan2(y,x);
        if (radJoystick<0) radJoystick+=(2*Math.PI);
        double radHeading = heading/(180/Math.PI)+(Math.PI/2);
        if (radHeading<0) radHeading+=(2*Math.PI);

        double targetHeading = radJoystick+(Math.PI/2-radHeading);
        double square = ((Math.sqrt(2)-1)*(Math.abs(Math.sin(2*targetHeading))))+1;

        frontrightmotorpower = (Math.sin(targetHeading)-Math.cos(targetHeading))/square;
        frontleftmotorpower = (Math.sin(targetHeading)+Math.cos(targetHeading))/square;
        backrightmotorpower = (Math.sin(targetHeading)+Math.cos(targetHeading))/square;
        backleftmotorpower = (Math.sin(targetHeading)-Math.cos(targetHeading))/square;


        //Something else maybe. If it doesn't work, just copy these things back to where they came from.
        frontrightmotorpower = (0.7*(speed*frontrightmotorpower)-.3*turn);
        frontleftmotorpower = (0.7*(speed*frontleftmotorpower)+.3*turn);
        backrightmotorpower = (0.7*(speed*backrightmotorpower)-.3*turn);
        backleftmotorpower = (0.7*(speed*backleftmotorpower)+.3*turn);


        //Speed equalizer maybe
        double [] powerarray = {Math.abs(frontleftmotorpower), Math.abs(frontrightmotorpower), Math.abs(backleftmotorpower), Math.abs(backrightmotorpower)};
        Arrays.sort(powerarray);

        double max = powerarray[powerarray.length - 1];

        frontleftmotorpower = frontleftmotorpower/max;
        frontrightmotorpower = frontrightmotorpower/max;
        backleftmotorpower = backleftmotorpower/max;
        backrightmotorpower = backrightmotorpower/max;

        //End of equalizer maybe

        if (gamepad1.dpad_up) {
            basePosition=380;
            boxServoPosition=0.65;
        }
        else if (gamepad1.dpad_right) {
            basePosition=215;
            boxServoPosition=0.5;
        }
        else if (gamepad1.dpad_down) {
            basePosition=25;
            boxServoPosition=0.1;
        }

        if (gamepad1.right_stick_button) {
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

        if (gamepad1.a) boxServoPosition=0.65;
        else if (gamepad1.b) boxServoPosition=0.50;
        else if (gamepad1.y) boxServoPosition=0.1;

        if (gamepad1.x) {
            if (spin==0) {
                spinMotorPower = 1;
                spin++;
            }
            else {
                spinMotorPower = 0;
                spin=0;
            }
        }

        if (gamepad2.left_bumper) boxServoCorrection-=0.005;
        else if (gamepad2.right_bumper) boxServoCorrection+=0.005;

        if (gamepad2.left_trigger>0.1) basePosition+=2;
        if (gamepad1.right_trigger>0.1) basePosition-=2;

        if (gamepad1.left_bumper) {
            if (basePosition==380) {
                if (trapDoorPosition!=.75) trapDoorPosition=.75;
                else trapDoorPosition=.35;
            }
            else if (basePosition==215) {
                if (trapDoorPosition!=.55) trapDoorPosition=.55;
                else trapDoorPosition=0.35;
            }
            else trapDoorPosition=0.35;
        }

        if (gamepad1.right_bumper && scale==0) {
            if (speedScale!=1) speedScale=1;
            else speedScale=0.5;
            scale++;
        }
        else scale=0;

        if (gamepad1.right_trigger>0.1) {
            basePosition=25;
            boxServoPosition=0.50;
        }


        baseMotor.setTargetPosition(basePosition);
        baseMotor.setPower(1);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spinMotor.setPower(spinMotorPower);

        frontrightmotor.setPower(frontrightmotorpower*speedScale);
        frontleftmotor.setPower(frontleftmotorpower*speedScale);
        backrightmotor.setPower(backrightmotorpower*speedScale);
        backleftmotor.setPower(backleftmotorpower*speedScale);

        topMotor.setPower(gamepad1.right_trigger-.3*gamepad1.left_trigger);
        boxServo.setPosition(boxServoPosition + boxServoCorrection);
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
