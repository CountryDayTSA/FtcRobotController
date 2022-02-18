package org.firstinspires.ftc.teamcode.MethodLibrary2021;

//This is the new Method Library for the 2019 - 2020 robotics competition
//everything should bet set up for a mech drive
//2/16/2020 - now uses gyroencoderdrive

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CountDownLatch;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class Something extends LinearOpMode {

    ////////////////////////// INSTANTIATING MOTORS AND SERVOS ////////////////////////////////////
    DcMotor frontleftmotor, frontrightmotor, backleftmotor, backrightmotor, leftbottomcrank,
            rightbottomcrank, leftintakemotor, rightintakemotor;
    Servo rightfoundationservo, leftfoundationservo, intakeservo, savox, intakeassistservo;
    DistanceSensor leftrangesensor, rightrangesensor, intakedistancesensor;
    ColorSensor downcolorsensor;

    //////////////////////////////// IMU INSTANTIATING ////////////////////////////////////////////
    BNO055IMU imu;
    Orientation angles = new Orientation();
    int relativeLayoutId;

    //////////////////////////////////////// PIDF INSTANTIATIONS ///////////////////////////////////

    public static final double NEW_P = 4.5;
    public static final double NEW_I = 3.5;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 5.0;


    //////////////////////////////// OTHER INSTANTIATIONS //////////////////////////////////////////
    Telemetry write = new TelemetryImpl(this);
    ElapsedTime runtime = new ElapsedTime();
    public double positionfromcenter;

    /////////////////////////////////////// METHODS ////////////////////////////////////////////////


    public void hardwareMap() {
        /*
        This method uses allows us to initialize all of our motors, servos, and sensors with one
        command. Therefore it saves us a lot of time while coding so that we don't have to copy and
        paste our messy hardware mapping into each autonomous code that we make.
         */
        frontleftmotor = hardwareMap.dcMotor.get("FLM");
        //frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor = hardwareMap.dcMotor.get("FRM");
        //frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor = hardwareMap.dcMotor.get("BLM");
        //backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor = hardwareMap.dcMotor.get("BRM");
        //backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bottomServo = hardwareMap.servo.get("BS");
        rightfoundationservo = hardwareMap.servo.get("RFS");
        leftfoundationservo = hardwareMap.servo.get("LFS");


        downcolorsensor = hardwareMap.get(ColorSensor.class, "DCS");
        intakedistancesensor = hardwareMap.get(DistanceSensor.class, "IDS");
        rightrangesensor = hardwareMap.get(DistanceSensor.class, "RRS");
        leftrangesensor = hardwareMap.get(DistanceSensor.class, "LRS");

        leftbottomcrank = hardwareMap.dcMotor.get("LBC");
        rightbottomcrank = hardwareMap.dcMotor.get("RBC");
        intakeservo = hardwareMap.servo.get("BS");
        savox = hardwareMap.servo.get("SAVOX");
        leftintakemotor = hardwareMap.dcMotor.get("LIM");
        rightintakemotor = hardwareMap.dcMotor.get("RIM");
        leftbottomcrank = hardwareMap.get(DcMotor.class, "LBC");
        leftbottomcrank.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbottomcrank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbottomcrank = hardwareMap.get(DcMotor.class, "RBC");
        rightbottomcrank.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbottomcrank.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbottomcrank.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorControllerEx leftbottomcrankcontroller = (DcMotorControllerEx) leftbottomcrank.getController();
        DcMotorControllerEx rightbottomcrankcontroller = (DcMotorControllerEx) rightbottomcrank.getController();
        int leftbottomcrankIndex = (leftbottomcrank).getPortNumber();
        int rightbottomcrankIndex = (rightbottomcrank).getPortNumber();
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        leftbottomcrankcontroller.setPIDFCoefficients(leftbottomcrankIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        rightbottomcrankcontroller.setPIDFCoefficients(rightbottomcrankIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        intakeassistservo = hardwareMap.servo.get("IAS");


        frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backleftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frm and blm were reversed before, so that means I needed to reverse brm, blm, and flm

        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //hardware mapping for the gyroscope
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());


    }

    public void waitSec(double seconds) {
        /*
        Create a separate timer so that we can time both the total run time and this method's.
         */
        ElapsedTime waitSecTime = new ElapsedTime();
        waitSecTime.reset();
        while (opModeIsActive() & waitSecTime.seconds() < seconds) {
            System.out.println("(-(-(- -)-)-)");
        }
    }

    private double reduceValues (double maxmin, double flm, double frm, double blm, double brm) {
        /*
        This method reduces the values of all the motors while maintaining the ratio

         */
        if (Math.abs(flm) > maxmin || Math.abs(frm) > maxmin || Math.abs(blm) > maxmin || Math.abs(brm) > maxmin) {
            if (Math.abs(flm) > maxmin) {
                flm = flm/(Math.abs(flm));
                frm = frm/(Math.abs(flm));
                blm = blm/(Math.abs(flm));
                brm = brm/(Math.abs(flm));
            } else if (Math.abs(frm) > maxmin) {
                flm = flm/(Math.abs(frm));
                frm = frm/(Math.abs(frm));
                blm = blm/(Math.abs(frm));
                brm = brm/(Math.abs(frm));
            } else if (Math.abs(blm) > maxmin) {
                flm = flm/(Math.abs(blm));
                frm = frm/(Math.abs(blm));
                blm = blm/(Math.abs(blm));
                brm = brm/(Math.abs(blm));
            } else {
                flm = flm/(Math.abs(brm));
                frm = frm/(Math.abs(brm));
                blm = blm/(Math.abs(brm));
                brm = brm/(Math.abs(brm));
            }
        }

        return flm + frm + blm + brm;
    }

    public void stopAndResetEncoders () {
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders () {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runUsingEncoders () {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public float getHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    public int get360heading() {
        int heading = (int) getHeading();
        if (heading < 0) heading += 360;
        return heading;
    }

    public void resetGyro() {
        /*
        sometimes our gyro doesn't work for an odd reason, so we keep this in case we need a
        temporary fix.
         */

        BNO055IMU imu;

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    }

    public void gyrodrive(double speed, String direction, double initialHeading) {

        /*
        and now we've made it to the gyro drive code. As you might notice here that the gyro drive
        is not in a loop. this is because this was designed to be used in the vuforia approach
        which already uses a loop. Therefore, we don't use a loop because we don't want to use a
        loop within a loop.
         */

        double accuracy = .5;
        double attack = .01;
        double distance;

        double frontleftmotorpower = speed;
        double frontrightmotorpower = speed;
        double backleftmotorpower = speed;
        double backrightmotorpower = speed;

        /*
        So what we do is we collect the actual heading that the robot is at, and we subtract it
        from the initial heading to get the heading that our robot should be at this moment.
         */

        double actualHeading = getHeading();
        double idealHeading = (actualHeading - initialHeading);
        distance = Math.abs(idealHeading);

        /*
        Having calculated the distance from the initial heading we can multiply the distance by the
        attack, which is a variable we can tune, and either add or subtract it from the motor power.
        Depending on whether its to the right or the left of the target heading.
         */

        if (idealHeading > accuracy) {
            telemetry.addLine("left of target");
            frontleftmotorpower += attack * distance;
            frontrightmotorpower -= attack * distance;
            backleftmotorpower += attack * distance;
            backrightmotorpower -= attack * distance;
        } else if (idealHeading < -accuracy) {
            telemetry.addLine("right of target");
            frontleftmotorpower -= attack * distance;
            frontrightmotorpower += attack * distance;
            backleftmotorpower -= attack * distance;
            backrightmotorpower += attack * distance;
        } else {
            frontleftmotorpower = speed;
            frontrightmotorpower = speed;
            backleftmotorpower = speed;
            backrightmotorpower = speed;
        }

        /*
        Instead of calculating the correct positive and negatives for each direction, we can just
        use the same powers and switch the motors that they're powering. We multiplied the powers
        when moving to the left and right because perpendicular movements are slower due to the
        mechanum drivetrain.
         */

        if (direction.equals("forward")) {

            frontleftmotor.setPower(frontleftmotorpower);
            frontrightmotor.setPower(-frontrightmotorpower);
            backleftmotor.setPower(backleftmotorpower);
            backrightmotor.setPower(-backrightmotorpower);

            telemetry.addData("FLM: ", frontleftmotorpower);
            telemetry.addData("FRM: ", frontrightmotorpower);
            telemetry.addData("BLM: ", backleftmotorpower);
            telemetry.addData("BRM: ", backrightmotorpower);

        } else if (direction.equals("right")) {

            frontleftmotor.setPower(backleftmotorpower * 1.5);
            frontrightmotor.setPower(frontleftmotorpower * 1.5);
            backleftmotor.setPower(-backrightmotorpower * 1.5);
            backrightmotor.setPower(-frontrightmotorpower * 1.5);

            telemetry.addData("FLM: ", backleftmotorpower * 1.5);
            telemetry.addData("FRM: ", frontleftmotorpower * 1.5);
            telemetry.addData("BLM: ", backrightmotorpower * 1.5);
            telemetry.addData("BRM: ", frontrightmotorpower * 1.5);

        } else if (direction.equals("left")) {

            frontleftmotor.setPower(-frontrightmotorpower * 1.5);
            frontrightmotor.setPower(-backrightmotorpower * 1.5);
            backleftmotor.setPower(frontleftmotorpower * 1.5);
            backrightmotor.setPower(backleftmotorpower * 1.5);

            telemetry.addData("FLM: ", frontrightmotorpower * 1.5);
            telemetry.addData("FRM: ", backrightmotorpower * 1.5);
            telemetry.addData("BLM: ", frontleftmotorpower * 1.5);
            telemetry.addData("BRM: ", backleftmotorpower * 1.5);

        } else {

            frontleftmotor.setPower(-backrightmotorpower);
            frontrightmotor.setPower(backleftmotorpower);
            backleftmotor.setPower(-frontrightmotorpower);
            backrightmotor.setPower(frontleftmotorpower);

            telemetry.addData("FLM: ", backrightmotorpower);
            telemetry.addData("FRM: ", backleftmotorpower);
            telemetry.addData("BLM: ", frontrightmotorpower);
            telemetry.addData("BRM: ", frontleftmotorpower);

        }

        telemetry.addData("InitHead: ", initialHeading);
        telemetry.addData("ActualHeading: ", actualHeading);
        telemetry.addData("IdealHeading: ", idealHeading);
        telemetry.addData("distance: ", distance);

        //telemetry.update();
    }

    public void gyroDiagDrivev2 (double speed, double degree, double targetheading) {

        /*
        As of 2/7/2020 this is our new code for almost every movement. This method allows us to move
        in any direction diagonally, while using the robot's current heading to not only maintain
        the correct orientation, but to actually finish the movement having reached a new heading.
        For example I could tell the robot to move at 45 degrees of its initial starting heading,
        and finish the movement at a heading of 180 degrees and the robot would basically do a half
        turn while maintaining a straight path at 45 degrees. Additionally, if the robot is bumped,
        and the heading is messed up, the robot is able to correct itself and speed up the side
        of the robot that is behind in the turn. It's quite a powerful code and I'm really happy
        that I was able to figure it out.
         */

        runWithoutEncoders();

        //getting our current heading of 0 to 360
        double currentheading = getHeading();
        if (currentheading < 0) currentheading += 360;

        //our rotational heading just takes into account where we are trying to go versus where we
        //are currently.
        double rotationalheading = degree - currentheading;
        if (rotationalheading < 0) rotationalheading += 360;

        //we use radians because they work out better generally
        double radiandirection = Math.toRadians(rotationalheading);

        //Setting the vectors for each wheel, unfortunately each wheel needs to have its own
        //independent vectors.
        double flm = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * speed);
        double frm = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * speed);
        double blm = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * speed);
        double brm = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * speed);

        //We convert our current heading to radians and set a target in radians too.
        if (currentheading < 0) currentheading += 360;
        double radheading = Math.toRadians(currentheading);
        double radtargetheading = Math.toRadians(targetheading);
        double correction = 0;

        //Our attack allows us to tune how aggressively our robot turns. A lower attach has a
        //faster turn speed.
        double attack = (Math.PI/2);

        //We calculate the distances for what would be a left and what would be a right turn, and
        //we choose the fastest turn path to the ideal final heading.
        double radleftdistance = radtargetheading - radheading;
        if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

        double radrightdistance = radheading - radtargetheading;
        if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

        //we set the correction
        if (radrightdistance <= radleftdistance) {
            correction = radrightdistance/attack;
        } else {
            correction = -(radleftdistance/attack);
        }

        //and we apply the correction to the motors
        flm = flm + correction;
        frm = frm + correction;
        blm = blm + correction;
        brm = brm + correction;

        //If we just sent these powers to the motors we would be "cropping" the motor powers off at
        //one, but some of the powers, when everything is all added together are actually greater
        //than one. So rather than crop the values and destroy or motor ratios, we redude all the
        //motor powers to 1 and -1 by creating a multiplier and applying that to all of the motors.
        double powerReducer = 1;

        //We create an array of motor power and compare just the extremes
        double [] numbers = {flm, frm, blm, brm};

        Arrays.sort(numbers);

        double min = numbers[0];
        double max = numbers[numbers.length - 1];

        if ((Math.abs(max) > 1) || (Math.abs(min) > 1)) {
            if (Math.abs(max) >= Math.abs(min)) {
                powerReducer = (1/max);
            } else {
                powerReducer = (1/Math.abs(min));
            }
        };

        //applying the power reducer
        flm = flm * powerReducer;
        frm = frm * powerReducer;
        blm = blm * powerReducer;
        brm = brm * powerReducer;

        //setting motor power
        frontleftmotor.setPower(flm);
        backleftmotor.setPower(blm);
        frontrightmotor.setPower(frm);
        backrightmotor.setPower(brm);

        //this is just so that I could read the telemetry easier, I'm rounding the decimals
        flm = Math.round(flm * 1000.0)/1000.0;
        frm = Math.round(frm * 1000.0)/1000.0;
        blm = Math.round(blm * 1000.0)/1000.0;
        brm = Math.round(brm * 1000.0)/1000.0;

        //telemetry
        telemetry.addLine("flm " + flm);
        telemetry.addLine("frm " + frm);
        telemetry.addLine("blm " + blm);
        telemetry.addLine("brm " + brm);
    }

    public void relativeGyroTurn(float degrees, String direction, double accuracy) {

        /*
        Welcome to the relative gyro turn code. It's a little hectic and it has taken me a long time
        to actually get this one working, but I'm quite proud of it. Last year I first made a
        program that did the turn, but when it overshot the target degree it would do an extra
        circle. I had been trying to build on that code to correct its turn backwards if it
        overshot the target because that is a lot faster than doing a whole extra turn or spending
        hours trying to tune the turn to work perfectly at every distance. Basically what this does
        is by creating an ideal heading which is the difference between the actual heading and the
        initial heading. As the ideal heading increases towards the value of the degree, the
        distance will decrease and decelerate the robot.
         */

        runtime.reset();
        stopAndResetEncoders();
        runWithoutEncoders();

        if (!direction.equals("left") && !direction.equals("right")) {
            /*
            In case "left" or "right" was misspelled, then we hope that they wanted left...
             */
            telemetry.addLine("Direction input invalid, defaulting to left...");
            direction = "left";
            telemetry.update();
            waitSec(1);
            telemetry.clearAll();
        }

        /*
        In case someone thinks it funny to try and input a negative degree, we will convert it to
        be positive.
         */

        if (degrees < 0) degrees +=360;

        if (direction.equals("left")) {

            /*
            we created a variable to manage the speed of the robot which should decelerate as the
            heading approaches the target heading. using the  heading we set the initial heading
            and at this point the ideal heading will be equal to zero and the distance will just be
            equal to the input degrees.
             */

            double speed;
            double actualHeading = getHeading();
            if (actualHeading < 0) actualHeading += 360;
            double initialHeading = actualHeading;
            double idealHeading = 0;
            double distance = degrees - idealHeading;

            while (opModeIsActive() && (distance >= accuracy || distance <= -accuracy)) {

                /*
                In the while loop is where the magic happens. We set an actual heading equal to the
                current heading of the robot and convert it from a -180 to 180 scale to a 0 to 360
                scale so to avoid the gap over larger turns. Then we calculate in ideal heading
                which will always begin at zero and work its way towards the target heading or
                degree as the distance decreases.
                 */

                actualHeading = getHeading();
                if (actualHeading < 0) actualHeading += 360;
                idealHeading = actualHeading - initialHeading;
                if (idealHeading < 0) idealHeading += 360;
                distance = degrees - idealHeading;

                if ((runtime.seconds() < .5)) {
                    distance = Math.abs(distance);
                }

                /*
                if the distance is positive than the robot will turn left, and once the distance
                becomes negative the robot will turn in the opposite direction. The distance would
                be negative if the robot overturned and this is a nice feature that allows us to be
                more precise in our code.
                 */

                if (distance <= -accuracy) {
                    speed = (distance / 9.9) - .08;
                    if (speed <= -.1) speed = -.1;
                } else {
                    speed = (distance/110) + .1;
                    if (speed >= .30) speed = .30;
                }

                frontleftmotor.setPower(-speed);
                frontrightmotor.setPower(-speed);
                backleftmotor.setPower(-speed);
                backrightmotor.setPower(-speed);

                telemetry.addLine("Current Heading: "+getHeading()+ " Distance from Target: "+distance);
                telemetry.addLine("speed: "+speed);
                telemetry.addLine("idealHeading: "+idealHeading);
                telemetry.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                telemetry.update();
            }
            stopDrive();
         //   writeTelemetry("done turning");
        } else {

            /*
            This would be for a right turn and it just does the opposite of the left turn.
             */

            double actualHeading = getHeading();
            if (actualHeading <= 0) actualHeading += 360;
            double initialHeading = actualHeading;
            double idealHeading = 0;
            double distance = degrees - idealHeading;

            double speed;

            while (opModeIsActive() && (distance >= accuracy || distance <= -accuracy)) {

                actualHeading = getHeading();
                if (actualHeading <= 0) actualHeading += 360;
                idealHeading = actualHeading - initialHeading;
                if (idealHeading <= 0) idealHeading += 360;
                distance = degrees - (360 - idealHeading);

                if ((runtime.seconds() < .5)) {
                    distance = Math.abs(distance);
                }

                if (distance <= -accuracy) {
                    speed = (distance / 9.9) - .08;
                    if (speed <= -.1) speed = -.1;
                } else {
                    speed = (distance/110) + .08;
                    if (speed >= .30) speed = .30;
                }

                frontleftmotor.setPower(speed);
                frontrightmotor.setPower(speed);
                backleftmotor.setPower(speed);
                backrightmotor.setPower(speed);
                telemetry.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                telemetry.addLine("speed: "+speed);
                telemetry.addLine("idealHeading: "+idealHeading);
                telemetry.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                telemetry.update();
            }
            stopDrive();
      //      writeTelemetry("done turning");
        }
    }

    public void absoluteGyroTurn(float target, double accuracy) {

        /*
        this calculates the quickest turn to reach the target heading and inputs it into the
        relative gyro turn code.
         */

        float distanceLeft = getHeading() - target;
        float distanceRight = target - getHeading();
        if (distanceLeft < 0) distanceLeft += 360;
        if (distanceRight < 0) distanceRight += 360;
        if(distanceLeft <= distanceRight) {
            relativeGyroTurn(distanceLeft, "left", accuracy);
        } else {
            relativeGyroTurn(distanceRight, "right", accuracy);
        }
    }

    ///////////////////////////////////// DRIVE METHODS ///////////////////////////////////////////

    public void drive(double power) {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontleftmotor.setPower(power);
        frontrightmotor.setPower(-power);
        backleftmotor.setPower(power);
        backrightmotor.setPower(-power);
    }

    public void perpdrive(String direction, double power) {
        if (direction == "left") {
            frontleftmotor.setPower(-power);
            frontrightmotor.setPower(-power);
            backrightmotor.setPower(power);
            backleftmotor.setPower(power);
        } else {
            frontleftmotor.setPower(power);
            frontrightmotor.setPower(power);
            backrightmotor.setPower(-power);
            backleftmotor.setPower(-power);
        }
    }

    public void coastMotors() {
        frontleftmotor.setZeroPowerBehavior(FLOAT);
        frontrightmotor.setZeroPowerBehavior(FLOAT);
        backleftmotor.setZeroPowerBehavior(FLOAT);
        backrightmotor.setZeroPowerBehavior(FLOAT);
    }

    public void brakeMotors() {
        frontleftmotor.setZeroPowerBehavior(BRAKE);
        frontrightmotor.setZeroPowerBehavior(BRAKE);
        backleftmotor.setZeroPowerBehavior(BRAKE);
        backrightmotor.setZeroPowerBehavior(BRAKE);
    }

    public void stopDrive() {
        frontrightmotor.setPower(0);
        frontleftmotor.setPower(0);
        backrightmotor.setPower(0);
        backleftmotor.setPower(0);
    }

    public void diagDrive (double direction, double speed) {

        /*
        This is a drive that I am still playing with, but basically it sets each motors speed based
        on the angle that I am trying to drive towards.
         */

        double radiandirection = direction*(Math.PI/180);
        double flm;
        double frm;
        double blm;
        double brm;

        /*
        Assume that the cosine of the radian direction is controlling the forward direction and the
        sine of the radian direction is controlling the right and left movements. If the radian
        direction is equal to 2 pi or 0 the cosine will return a value of 1 and the wheels will be
        powered with that value multiplied by the speed.
         */

        flm = (Math.cos(radiandirection) + -Math.sin(radiandirection)) * speed;
        frm = (-Math.cos(radiandirection) + -Math.sin(radiandirection)) * speed;
        blm = (Math.cos(radiandirection) + Math.sin(radiandirection)) * speed;
        brm = (-Math.cos(radiandirection) + Math.sin(radiandirection))* speed;

        frontleftmotor.setPower(flm);
        backleftmotor.setPower(blm);
        frontrightmotor.setPower(frm);
        backrightmotor.setPower(brm);

        telemetry.addLine("flm " + flm);
        telemetry.addLine("frm " + frm);
        telemetry.addLine("blm " + blm);
        telemetry.addLine("brm " + brm);
        telemetry.update();

    }

    /////////////////////////////////// ENCODER DRIVE METHODS /////////////////////////////////////

    enum Wheel {
        flm, frm, blm, brm;
    }

    public double encoderdistancetravelled () {
        /*
        telemetry.addLine("frontLeftMotor: " + frontleftmotor.getCurrentPosition());
        telemetry.addLine("frontRightMotor: " + frontrightmotor.getCurrentPosition());
        telemetry.addLine("backLeftMotor: " + backleftmotor.getCurrentPosition());
        telemetry.addLine("backRightMotor: " + backrightmotor.getCurrentPosition());
        telemetry.update();

         */

        double averageposition = (Math.abs(frontleftmotor.getCurrentPosition()) + Math.abs(frontrightmotor.getCurrentPosition()) + Math.abs(backleftmotor.getCurrentPosition()) + Math.abs(backrightmotor.getCurrentPosition()))/4;

        return averageposition;
    }

    public int encoderdistancetravelledwheel (Wheel wheel) {

        int wheelposition = 0;

        if (wheel.equals(wheel.flm)) {
            wheelposition = frontleftmotor.getCurrentPosition();
        } else if (wheel.equals(wheel.frm)) {
            wheelposition = frontrightmotor.getCurrentPosition();
        } else if (wheel.equals(wheel.blm)) {
            wheelposition = backleftmotor.getCurrentPosition();
        } else if (wheel.equals(wheel.brm)) {
            wheelposition = backrightmotor.getCurrentPosition();
        }

        return wheelposition;
    }

    public double encodertargetdistance () {

        double averagetargetposition = (Math.abs(frontleftmotor.getTargetPosition()) + Math.abs(frontrightmotor.getTargetPosition()) + Math.abs(backleftmotor.getTargetPosition()) + Math.abs(backrightmotor.getTargetPosition()))/4;

        return averagetargetposition;
    }

    public double encodertargetdistancewheel (Wheel wheel) {

        double wheelposition = 0;

        if (wheel.equals(wheel.flm)) {
            wheelposition = frontleftmotor.getTargetPosition();
        } else if (wheel.equals(wheel.frm)) {
            wheelposition = frontrightmotor.getTargetPosition();
        } else if (wheel.equals(wheel.blm)) {
            wheelposition = backleftmotor.getTargetPosition();
        } else if (wheel.equals(wheel.brm)) {
            wheelposition = backrightmotor.getTargetPosition();
        }

        return wheelposition;
    }

    public void encoderDrive (double inches, double inputSpeed) {

        inches = inches + .18;
        //inputSpeed += .1;

        /* This is our main drive method, which uses the encoders to precisely calculate distance.
         * There is a bunch of stuff that we could do a lot more efficiently, but meh.
         */

        //537.6 counts per revolution
        //Wheel circumference is 12.57in
        //537.6/12.57 = 42.77 ticks per inch
        final double COUNTS_PER_INCH = 57;
        double speed = 0.75*inputSpeed;

        /*
        1. Change the motor mode to STOP_AND_RESET_ENCODER
        2. Set target values using setTargetPosition
        3. Change the motor mode to RUN_TO_POSITION
        4. Set a motor power (this actually sets a maximum/minimum power that the robot changes based on distance from the target)
         */



        stopAndResetEncoders();
        runUsingEncoders();

        //Reset the current encoder counts to 0;
        boolean frontLeftMotorStop = false;
        boolean frontRightMotorStop = false;
        boolean backLeftMotorStop = false;
        boolean backRightMotorStop = false;

        //Declare 4 booleans that are used to end the while() loop further down.
        int newLeftTarget = (int) (inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (inches * COUNTS_PER_INCH); /**/
        frontleftmotor.setTargetPosition(newLeftTarget);
        frontrightmotor.setTargetPosition(-newRightTarget);
        backleftmotor.setTargetPosition(newLeftTarget);
        backrightmotor.setTargetPosition(-newRightTarget);

        //Set a target distance for the motors using the pre-calculated int values.
        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        frontleftmotor.setPower(speed);
        frontrightmotor.setPower(-speed);
        backleftmotor.setPower(speed);
        backrightmotor.setPower(-speed);

        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !frontLeftMotorStop & !frontRightMotorStop
                & !backLeftMotorStop & !backRightMotorStop) {
            //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            //distance = inches - frontLeftMotor.getCurrentPosition()/COUNTS_PER_INCH;
            speed = inputSpeed;

            frontleftmotor.setPower(speed);
            frontrightmotor.setPower(-speed);
            backleftmotor.setPower(speed);
            backrightmotor.setPower(-speed);

            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("frontLeftMotor: " + frontleftmotor.getCurrentPosition() +
                    " of " + frontleftmotor.getTargetPosition());
            telemetry.addLine("frontRightMotor: " + frontrightmotor.getCurrentPosition() +
                    " of " + frontrightmotor.getTargetPosition());
            telemetry.addLine("backLeftMotor: " + backleftmotor.getCurrentPosition() +
                    " of " + backleftmotor.getTargetPosition());
            telemetry.addLine("backRightMotor: " + backrightmotor.getCurrentPosition() +
                    " of " + backrightmotor.getTargetPosition());
            telemetry.update();


            if ((encodertargetdistance() - encoderdistancetravelled()) <= 30) {
                frontLeftMotorStop = true;
                frontRightMotorStop = true;
                backLeftMotorStop = true;
                backRightMotorStop = true;
            }

        }
        frontleftmotor.setPower(0);
        frontrightmotor.setPower(0);
        backleftmotor.setPower(0);
        backrightmotor.setPower(0);
    }

    public void encoderGyroDrive (double inches, double inputSpeed, double initialheading) {

        inches = inches + .18;
        //inputSpeed += .1;
        /* This is our main drive method, which uses the encoders to precisely calculate distance.
         * There is a bunch of stuff that we could do a lot more efficiently, but meh.
         */

        final double COUNTS_PER_INCH = 57;
        double speed = 0.75*inputSpeed;

        /*
        The steps to create any encoder drive:
        1. Change the motor mode to STOP_AND_RESET_ENCODER
        2. Set target values using setTargetPosition
        3. Change the motor mode to RUN_TO_POSITION
        4. Set a motor power (this actually sets a maximum/minimum power that the robot changes based on distance from the target)
         */

        stopAndResetEncoders();
        runUsingEncoders();

        //Reset the current encoder counts to 0;
        boolean frontLeftMotorStop = false;
        boolean frontRightMotorStop = false;
        boolean backLeftMotorStop = false;
        boolean backRightMotorStop = false;

        //Declare 4 booleans that are used to end the while() loop further down.
        int newLeftTarget = (int) (inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (inches * COUNTS_PER_INCH); /**/
        frontleftmotor.setTargetPosition(newLeftTarget);
        frontrightmotor.setTargetPosition(-newRightTarget);
        backleftmotor.setTargetPosition(newLeftTarget);
        backrightmotor.setTargetPosition(-newRightTarget);

        //Set a target distance for the motors using the pre-calculated int values.
        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        frontleftmotor.setPower(speed);
        frontrightmotor.setPower(-speed);
        backleftmotor.setPower(speed);
        backrightmotor.setPower(-speed);

        while (opModeIsActive() & !frontLeftMotorStop & !frontRightMotorStop
                & !backLeftMotorStop & !backRightMotorStop) {
            //We use opModeIsActive() in many of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            speed = inputSpeed;


            /*
            if (inputSpeed > .2) {
                double difference = ((inches * COUNTS_PER_INCH)/encoderdistancetravelled());
                double multiplier = Math.pow(4, (difference-1.7));
                if (multiplier > 1) multiplier = 1;
                speed = inputSpeed*multiplier;
                if (speed < .08) speed = .08;
            } else {
                speed = inputSpeed;
            }

             */

            /*
            We are still toying with the concept of ramping down this encoder drive, so for the
            time being we set the input speed to equal the speed, but we will change this later.
            At the moment it just hasn't been a large concern.
             */


            /*
            Here we are incorporating the gyro drive into the encoder drive. Basically we can easily
            just use the gyro drive method to keep the robot straight here. Pretty simple and cool.
             */

            if (inches >= 0) {
                gyrodrive(speed, "forward", initialheading);
            } else {
                gyrodrive(speed, "backward", initialheading);
            }

            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("frontLeftMotor: " + frontleftmotor.getCurrentPosition() +
                    " of " + frontleftmotor.getTargetPosition());
            telemetry.addLine("frontRightMotor: " + frontrightmotor.getCurrentPosition() +
                    " of " + frontrightmotor.getTargetPosition());
            telemetry.addLine("backLeftMotor: " + backleftmotor.getCurrentPosition() +
                    " of " + backleftmotor.getTargetPosition());
            telemetry.addLine("backRightMotor: " + backrightmotor.getCurrentPosition() +
                    " of " + backrightmotor.getTargetPosition());
            telemetry.update();

            if ((encodertargetdistance() - encoderdistancetravelled()) <= 30) {
                frontLeftMotorStop = true;
                frontRightMotorStop = true;
                backLeftMotorStop = true;
                backRightMotorStop = true;
            }
        }
        //once the while loop is over we can turn off the motors.
        frontleftmotor.setPower(0);
        frontrightmotor.setPower(0);
        backleftmotor.setPower(0);
        backrightmotor.setPower(0);
    }

    public void encoderPerpendicularDrive (double inches, double inputSpeed, String direction) {
        inches = inches + .18;
        //inputSpeed += .1;
        /* This is our main drive method, which uses the encoders to precisely calculate distance.
         * There is a bunch of stuff that we could do a lot more efficiently, but meh.
         */

        final double COUNTS_PER_INCH = 62;
        //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double speed = inputSpeed;

        /*
        1. Change the motor mode to STOP_AND_RESET_ENCODER
        2. Set target values using setTargetPosition
        3. Change the motor mode to RUN_TO_POSITION
        4. Set a motor power (this actually sets a maximum/minimum power that the robot changes based on distance from the target)
         */

        stopAndResetEncoders();
        runUsingEncoders();

        //Reset the current encoder counts to 0;
        boolean frontLeftMotorStop = false;
        boolean frontRightMotorStop = false;
        boolean backLeftMotorStop = false;
        boolean backRightMotorStop = false;

        //Declare 4 booleans that are used to end the while() loop further down.
        //*Converts the input of inches into encoder counts*/
        int newTarget = (int) (inches * COUNTS_PER_INCH);
        if (!direction.equals("left") && !direction.equals("right")) {
            //Check if the String input for direction was neither "left" nor "right". If that is the case, it defaults to "left".
            telemetry.addLine("Direction input invalid, defaulting to left...");
            direction = "left";
            telemetry.update();
            //waitSec(1);
            telemetry.clearAll();
        }
        if (direction.equals("left")) {
            frontrightmotor.setTargetPosition(-newTarget);
            frontleftmotor.setTargetPosition(-newTarget);
            backrightmotor.setTargetPosition(newTarget);
            backleftmotor.setTargetPosition(newTarget);
            frontrightmotor.setPower(-speed);
            frontleftmotor.setPower(-speed);
            backrightmotor.setPower(speed);
            backleftmotor.setPower(speed);
            //Turn on the motors on so that they actually move. We forget that part a lot.
        } else {
            //direction must equal right
            frontrightmotor.setTargetPosition(newTarget);
            frontleftmotor.setTargetPosition(newTarget);
            backrightmotor.setTargetPosition(-newTarget);
            backleftmotor.setTargetPosition(-newTarget);
            frontrightmotor.setPower(speed);
            frontleftmotor.setPower(speed);
            backrightmotor.setPower(-speed);
            backleftmotor.setPower(-speed);
        }
        //Set a target distance for the motors using the pre-calculated int values.

        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.

        while (opModeIsActive() & !frontLeftMotorStop & !frontRightMotorStop
                & !backLeftMotorStop & !backRightMotorStop) {
            //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            speed = inputSpeed;

            if (direction.equals("left")) {
                frontrightmotor.setPower(-speed);
                frontleftmotor.setPower(-speed);
                backrightmotor.setPower(speed);
                backleftmotor.setPower(speed);
            } else {
                //right
                frontrightmotor.setPower(speed);
                frontleftmotor.setPower(speed);
                backrightmotor.setPower(-speed);
                backleftmotor.setPower(-speed);
            }
            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("frontLeftMotor: " + frontleftmotor.getCurrentPosition() +
                    " of " + frontleftmotor.getTargetPosition());
            telemetry.addLine("frontRightMotor: " + frontrightmotor.getCurrentPosition() +
                    " of " + frontrightmotor.getTargetPosition());
            telemetry.addLine("backLeftMotor: " + backleftmotor.getCurrentPosition() +
                    " of " + backleftmotor.getTargetPosition());
            telemetry.addLine("backRightMotor: " + backrightmotor.getCurrentPosition() +
                    " of " + backrightmotor.getTargetPosition());
            telemetry.update();


            if ((encodertargetdistance() - encoderdistancetravelled()) <= 30) {
                frontLeftMotorStop = true;
                frontRightMotorStop = true;
                backLeftMotorStop = true;
                backRightMotorStop = true;
            }
        }
        frontleftmotor.setPower(0);
        frontrightmotor.setPower(0);
        backleftmotor.setPower(0);
        backrightmotor.setPower(0);
    }

    public void encoderGyroPerpendicularDrive (double inches, double inputSpeed, String direction, double initialheading) {
        inches = inches + .18;
        //inputSpeed += .1;
        /*
        The only difference between this code and the regular gyro encoder drive is that this one
        is for perpendicular movements. We are curently in the process of creating a gyro encoder
        drive.
         */

        final double COUNTS_PER_INCH = 62;
        double speed = inputSpeed;

        /*
        1. Change the motor mode to STOP_AND_RESET_ENCODER
        2. Set target values using setTargetPosition
        3. Change the motor mode to RUN_TO_POSITION
        4. Set a motor power (this actually sets a maximum/minimum power that the robot changes based on distance from the target)
         */

        stopAndResetEncoders();
        runUsingEncoders();

        boolean frontLeftMotorStop = false;
        boolean frontRightMotorStop = false;
        boolean backLeftMotorStop = false;
        boolean backRightMotorStop = false;

        int newTarget = (int) (inches * COUNTS_PER_INCH);
        if (!direction.equals("left") && !direction.equals("right")) {
            telemetry.addLine("Direction input invalid, defaulting to left...");
            direction = "left";
            telemetry.update();
            //waitSec(1);
            telemetry.clearAll();
        }

        /*
        This is where the code changes depending on whether we're driving to the right or the left.
         */
        if (direction.equals("left")) {
            frontrightmotor.setTargetPosition(-newTarget);
            frontleftmotor.setTargetPosition(-newTarget);
            backrightmotor.setTargetPosition(newTarget);
            backleftmotor.setTargetPosition(newTarget);
            frontrightmotor.setPower(-speed);
            frontleftmotor.setPower(-speed);
            backrightmotor.setPower(speed);
            backleftmotor.setPower(speed);
        } else {
            //direction must equal right
            frontrightmotor.setTargetPosition(newTarget);
            frontleftmotor.setTargetPosition(newTarget);
            backrightmotor.setTargetPosition(-newTarget);
            backleftmotor.setTargetPosition(-newTarget);
            frontrightmotor.setPower(speed);
            frontleftmotor.setPower(speed);
            backrightmotor.setPower(-speed);
            backleftmotor.setPower(-speed);
        }

        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        Now that we have a target position we are setting the motors to run to that tick count.
         */

        while (opModeIsActive() & !frontLeftMotorStop & !frontRightMotorStop
                & !backLeftMotorStop & !backRightMotorStop) {
            speed = inputSpeed;

            /*
            if (inputSpeed > .2) {
                double difference = ((inches * COUNTS_PER_INCH)/encoderdistancetravelled());
                double multiplier = Math.pow(4, (difference-1.7));
                if (multiplier > 1) multiplier = 1;
                speed = inputSpeed*multiplier;
                if (speed < .08) speed = .08;
            } else {
                speed = inputSpeed;
            }
             */

            if (direction.equals("left")) {
                gyrodrive(speed, "left", initialheading);
            } else {
                gyrodrive(speed, "right", initialheading);
            }
            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+(speed));

            telemetry.addLine("frontLeftMotor: " + frontleftmotor.getCurrentPosition() +
                    " of " + frontleftmotor.getTargetPosition());
            telemetry.addLine("frontRightMotor: " + frontrightmotor.getCurrentPosition() +
                    " of " + frontrightmotor.getTargetPosition());
            telemetry.addLine("backLeftMotor: " + backleftmotor.getCurrentPosition() +
                    " of " + backleftmotor.getTargetPosition());
            telemetry.addLine("backRightMotor: " + backrightmotor.getCurrentPosition() +
                    " of " + backrightmotor.getTargetPosition());
            telemetry.update();


            if ((encodertargetdistance() - encoderdistancetravelled()) <= 30) {
                frontLeftMotorStop = true;
                frontRightMotorStop = true;
                backLeftMotorStop = true;
                backRightMotorStop = true;
            }
        }
        frontleftmotor.setPower(0);
        frontrightmotor.setPower(0);
        backleftmotor.setPower(0);
        backrightmotor.setPower(0);
    }

    public void encoderdiaggyrodrive (double inputspeed, double degree, double targetheading, double inches) {

        /*
        DcMotorControllerEx flmcontroller = (DcMotorControllerEx)frontleftmotor.getController();
        DcMotorControllerEx frmcontroller = (DcMotorControllerEx) frontrightmotor.getController();
        DcMotorControllerEx blmcontroller = (DcMotorControllerEx)frontleftmotor.getController();
        DcMotorControllerEx brmcontroller = (DcMotorControllerEx) frontrightmotor.getController();
        int flmIndex = (frontleftmotor).getPortNumber();
        int frmIndex = (frontrightmotor).getPortNumber();
        int blmIndex = (backleftmotor).getPortNumber();
        int brmIndex = (backrightmotor).getPortNumber();
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flmcontroller.setPIDFCoefficients(flmIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        frmcontroller.setPIDFCoefficients(frmIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        blmcontroller.setPIDFCoefficients(blmIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        brmcontroller.setPIDFCoefficients(brmIndex, DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

         */

        final double COUNTS_PER_RAD = (2200/Math.PI);
        final double COUNTS_PER_INCH = 57;
        double ticks = inches * COUNTS_PER_INCH;

        stopAndResetEncoders();
        runUsingEncoders();
        //runWithoutEncoders();

        double flmspeed = 0;
        double frmspeed = 0;
        double blmspeed = 0;
        double brmspeed = 0;

        boolean flmstop = false;
        boolean frmstop = false;
        boolean blmstop = false;
        boolean brmstop = false;

        double flmdifference = 0;
        double frmdifference = 0;
        double blmdifference = 0;
        double brmdifference = 0;

        double turningticks;




        double radheading = Math.toRadians(get360heading());

        double radtargetheading = Math.toRadians(targetheading);

        double rotationalheading = degree - get360heading();
        if (rotationalheading < 0) rotationalheading += 360;

        double radiandirection = Math.toRadians(rotationalheading);

        double flmdistance = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double frmdistance = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double blmdistance = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);
        double brmdistance = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);


        double radleftdistance = radtargetheading - radheading;
        if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

        double radrightdistance = radheading - radtargetheading;
        if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

        if (radrightdistance <= radleftdistance) {
            turningticks = (radrightdistance * COUNTS_PER_RAD);
        } else {
            turningticks = -(radleftdistance * COUNTS_PER_RAD);
        }

        frontleftmotor.setTargetPosition((int) (flmdistance + turningticks));
        frontrightmotor.setTargetPosition((int) (frmdistance + turningticks));
        backleftmotor.setTargetPosition((int) (blmdistance + turningticks));
        backrightmotor.setTargetPosition((int) (brmdistance + turningticks));

        int flmtargetposition = frontleftmotor.getTargetPosition();
        int frmtargetposition = frontrightmotor.getTargetPosition();
        int blmtargetposition = backleftmotor.getTargetPosition();
        int brmtargetposition = backrightmotor.getTargetPosition();

        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleftmotor.setPower(flmspeed);
        frontrightmotor.setPower(frmspeed);
        backleftmotor.setPower(blmspeed);
        backrightmotor.setPower(brmspeed);


        while (opModeIsActive() && (!flmstop || !frmstop || !blmstop || !brmstop)) {

            double flmposition = frontleftmotor.getCurrentPosition();
            double frmposition = frontrightmotor.getCurrentPosition();
            double blmposition = backleftmotor.getCurrentPosition();
            double brmposition = backrightmotor.getCurrentPosition();

            flmdifference = Math.abs(flmtargetposition) - Math.abs(flmposition);
            frmdifference = Math.abs(frmtargetposition) - Math.abs(frmposition);
            blmdifference = Math.abs(blmtargetposition) - Math.abs(blmposition);
            brmdifference = Math.abs(brmtargetposition) - Math.abs(brmposition);



            /*

            rotationalheading = degree - get360heading();
            if (rotationalheading < 0) rotationalheading += 360;

            radiandirection = Math.toRadians(rotationalheading);

            flmspeed = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * inputspeed);
            frmspeed = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * inputspeed);
            blmspeed = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * inputspeed);
            brmspeed = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * inputspeed);

             */



            flmspeed = Math.sqrt(.005 * Math.abs(flmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            frmspeed = Math.sqrt(.005 * Math.abs(frmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            blmspeed = Math.sqrt(.005 * Math.abs(blmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            brmspeed = Math.sqrt(.005 * Math.abs(brmdifference)) + Math.abs(Math.sin(radiandirection) * .1);


            double powerReducer = 1;

            //We create an array of motor power and compare just the extremes
            double [] numbers = {flmspeed, frmspeed, blmspeed, brmspeed};

            Arrays.sort(numbers);

            double min = numbers[0];
            double max = numbers[numbers.length - 1];

            if ((Math.abs(max) > inputspeed) || (Math.abs(min) > inputspeed)) {
                if (Math.abs(max) >= Math.abs(min)) {
                    powerReducer = (inputspeed/max);
                } else {
                    powerReducer = (inputspeed/Math.abs(min));
                }
            };

            //applying the power reducer
            flmspeed = flmspeed * powerReducer;
            frmspeed = frmspeed * powerReducer;
            blmspeed = blmspeed * powerReducer;
            brmspeed = brmspeed * powerReducer;


            if (Math.abs(flmdifference) < 5) {
                flmstop = true;
                flmspeed = 0;
            } else {
                telemetry.addLine("FLMspeed " + flmspeed);
                //telemetry.addLine("flmposition " + flmposition);
                telemetry.addLine("FLMdifference " + flmdifference);
            }
            if (Math.abs(frmdifference) < 5) {
                frmstop = true;
                frmspeed = 0;
            } else {
                telemetry.addLine("FRMspeed " + frmspeed);
                //telemetry.addLine("frmposition " + frmposition);
                telemetry.addLine("FRMdifference " + frmdifference);
            }
            if (Math.abs(blmdifference) < 5) {
                blmstop = true;
                blmspeed = 0;
            } else {
                telemetry.addLine("BLMspeed " + blmspeed);
                //telemetry.addLine("blmposition " + blmposition);
                telemetry.addLine("BLMdifference " + blmdifference);
            }
            if (Math.abs(brmdifference) < 5) {
                brmstop = true;
                brmspeed = 0;
            } else {
                telemetry.addLine("BRMspeed " + brmspeed);
                //telemetry.addLine("brmposition " + brmposition);
                telemetry.addLine("BRMdifference " + brmdifference);
            }

            frontleftmotor.setPower(flmspeed);
            frontrightmotor.setPower(frmspeed);
            backleftmotor.setPower(blmspeed);
            backrightmotor.setPower(brmspeed);

            telemetry.update();
        }
        telemetry.addLine("done");
        stopDrive();
    }

    public void encoderdiaggyrodrivev2 (double inputspeed, double degree, double targetheading, double inches, Wheel wheel) {

        final double COUNTS_PER_RAD = (2200/Math.PI);
        final double COUNTS_PER_INCH = 57;
        double ticks = inches * COUNTS_PER_INCH;

        if (wheel.equals(wheel.flm)) {
            frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (wheel.equals(wheel.frm)) {
            frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (wheel.equals(wheel.blm)) {
            backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (wheel.equals(wheel.brm)) {
            backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double flmspeed = 0;
        double frmspeed = 0;
        double blmspeed = 0;
        double brmspeed = 0;

        boolean stop = false;

        double flmdifference = 0;
        double frmdifference = 0;
        double blmdifference = 0;
        double brmdifference = 0;

        double turningticks;



        double radheading = Math.toRadians(get360heading());

        double radtargetheading = Math.toRadians(targetheading);

        double rotationalheading = degree - get360heading();
        if (rotationalheading < 0) rotationalheading += 360;

        double radiandirection = Math.toRadians(rotationalheading);

        double flmdistance = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double frmdistance = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double blmdistance = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);
        double brmdistance = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);


        double radleftdistance = radtargetheading - radheading;
        if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

        double radrightdistance = radheading - radtargetheading;
        if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

        if (radrightdistance <= radleftdistance) {
            turningticks = (radrightdistance * COUNTS_PER_RAD);
        } else {
            turningticks = -(radleftdistance * COUNTS_PER_RAD);
        }

        frontleftmotor.setTargetPosition((int) (flmdistance + turningticks));
        frontrightmotor.setTargetPosition((int) (frmdistance + turningticks));
        backleftmotor.setTargetPosition((int) (blmdistance + turningticks));
        backrightmotor.setTargetPosition((int) (brmdistance + turningticks));

        int flmtargetposition = frontleftmotor.getTargetPosition();
        int frmtargetposition = frontrightmotor.getTargetPosition();
        int blmtargetposition = backleftmotor.getTargetPosition();
        int brmtargetposition = backrightmotor.getTargetPosition();

        if (wheel.equals(wheel.flm)) {
            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (wheel.equals(wheel.frm)) {
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (wheel.equals(wheel.blm)) {
            backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (wheel.equals(wheel.brm)) {
            backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        frontleftmotor.setPower(flmspeed);
        frontrightmotor.setPower(frmspeed);
        backleftmotor.setPower(blmspeed);
        backrightmotor.setPower(brmspeed);


        while (opModeIsActive() && (!stop)) {

            double flmposition = frontleftmotor.getCurrentPosition();
            double frmposition = frontrightmotor.getCurrentPosition();
            double blmposition = backleftmotor.getCurrentPosition();
            double brmposition = backrightmotor.getCurrentPosition();

            flmdifference = Math.abs(flmtargetposition) - Math.abs(flmposition);
            frmdifference = Math.abs(frmtargetposition) - Math.abs(frmposition);
            blmdifference = Math.abs(blmtargetposition) - Math.abs(blmposition);
            brmdifference = Math.abs(brmtargetposition) - Math.abs(brmposition);


            rotationalheading = degree - get360heading();
            if (rotationalheading < 0) rotationalheading += 360;

            radiandirection = Math.toRadians(rotationalheading);

            flmspeed = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * inputspeed);
            frmspeed = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * inputspeed);
            blmspeed = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * inputspeed);
            brmspeed = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * inputspeed);



            radheading = Math.toRadians(get360heading());
            radtargetheading = Math.toRadians(targetheading);
            double correction = 0;

            //Our attack allows us to tune how aggressively our robot turns. A lower attach has a
            //faster turn speed.
            double attack = (Math.PI/2);

            //We calculate the distances for what would be a left and what would be a right turn, and
            //we choose the fastest turn path to the ideal final heading.
            radleftdistance = radtargetheading - radheading;
            if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

            radrightdistance = radheading - radtargetheading;
            if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

            //we set the correction
            if (radrightdistance <= radleftdistance) {
                correction = radrightdistance/attack;
            } else {
                correction = -(radleftdistance/attack);
            }

            //and we apply the correction to the motors
            flmspeed = flmspeed + correction;
            frmspeed = frmspeed + correction;
            blmspeed = blmspeed + correction;
            brmspeed = brmspeed + correction;





            double powerReducer = 1;

            //We create an array of motor power and compare just the extremes
            double [] numbers = {flmspeed, frmspeed, blmspeed, brmspeed};

            Arrays.sort(numbers);

            double min = numbers[0];
            double max = numbers[numbers.length - 1];

            if ((Math.abs(max) > inputspeed) || (Math.abs(min) > inputspeed)) {
                if (Math.abs(max) >= Math.abs(min)) {
                    powerReducer = (inputspeed/max);
                } else {
                    powerReducer = (inputspeed/Math.abs(min));
                }
            }

            //applying the power reducer
            flmspeed = flmspeed * powerReducer;
            frmspeed = frmspeed * powerReducer;
            blmspeed = blmspeed * powerReducer;
            brmspeed = brmspeed * powerReducer;


            if (wheel.equals(wheel.flm)) {
                if (!frontleftmotor.isBusy()) {
                    stop = true;
                } else {
                    telemetry.addLine("FLMspeed " + flmspeed);
                    //telemetry.addLine("flmposition " + flmposition);
                    telemetry.addLine("FLMdifference " + flmdifference);
                }
            } else if (wheel.equals(wheel.frm)) {
                if (!frontrightmotor.isBusy()) {
                    stop = true;
                } else {
                    telemetry.addLine("FRMspeed " + frmspeed);
                    //telemetry.addLine("flmposition " + flmposition);
                    telemetry.addLine("FRMdifference " + frmdifference);
                }
            } else if (wheel.equals(wheel.blm)) {
                if (!backleftmotor.isBusy()) {
                    stop = true;
                } else {
                    telemetry.addLine("BLMspeed " + blmspeed);
                    //telemetry.addLine("frmposition " + frmposition);
                    telemetry.addLine("BLMdifference " + blmdifference);
                }
            } else if (wheel.equals(wheel.brm)) {
                if (!backrightmotor.isBusy()) {
                    stop = true;
                } else {
                    telemetry.addLine("BRMspeed " + brmspeed);
                    //telemetry.addLine("brmposition " + brmposition);
                    telemetry.addLine("BRMdifference " + brmdifference);
                }
            }


            frontleftmotor.setPower(flmspeed);
            frontrightmotor.setPower(frmspeed);
            backleftmotor.setPower(blmspeed);
            backrightmotor.setPower(brmspeed);

            telemetry.update();
        }
        telemetry.addLine("done");
        stopDrive();
    }

    public void encoderdiaggyrodrivev3 (double inputspeed, double degree, double targetheading, double inches, Wheel wheel) {

        final double COUNTS_PER_RAD = (2200/Math.PI);
        final double COUNTS_PER_INCH = 57;
        double ticks = inches * COUNTS_PER_INCH;

        stopAndResetEncoders();
        runWithoutEncoders();

        double flmspeed = 0;
        double frmspeed = 0;
        double blmspeed = 0;
        double brmspeed = 0;

        boolean stop = false;

        double flmdifference = 0;
        double frmdifference = 0;
        double blmdifference = 0;
        double brmdifference = 0;

        double turningticks;

        double wheeldifference = 0;



        double radheading = Math.toRadians(get360heading());

        double radtargetheading = Math.toRadians(targetheading);

        double rotationalheading = degree - get360heading();
        if (rotationalheading < 0) rotationalheading += 360;

        double radiandirection = Math.toRadians(rotationalheading);

        double flmdistance = ((Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double frmdistance = ((-Math.cos(radiandirection) - Math.sin(radiandirection)) * ticks);
        double blmdistance = ((Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);
        double brmdistance = ((-Math.cos(radiandirection) + Math.sin(radiandirection)) * ticks);


        double radleftdistance = radtargetheading - radheading;
        if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

        double radrightdistance = radheading - radtargetheading;
        if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

        if (radrightdistance <= radleftdistance) {
            turningticks = (radrightdistance * COUNTS_PER_RAD);
        } else {
            turningticks = -(radleftdistance * COUNTS_PER_RAD);
        }

        frontleftmotor.setTargetPosition((int) (flmdistance + turningticks));
        frontrightmotor.setTargetPosition((int) (frmdistance + turningticks));
        backleftmotor.setTargetPosition((int) (blmdistance + turningticks));
        backrightmotor.setTargetPosition((int) (brmdistance + turningticks));

        int flmtargetposition = frontleftmotor.getTargetPosition();
        int frmtargetposition = frontrightmotor.getTargetPosition();
        int blmtargetposition = backleftmotor.getTargetPosition();
        int brmtargetposition = backrightmotor.getTargetPosition();

        frontleftmotor.setPower(flmspeed);
        frontrightmotor.setPower(frmspeed);
        backleftmotor.setPower(blmspeed);
        backrightmotor.setPower(brmspeed);

        while (opModeIsActive() && (!stop)) {

            if (wheel.equals(Wheel.flm)) {
                double flmposition = frontleftmotor.getCurrentPosition();
                flmdifference = Math.abs(flmtargetposition) - Math.abs(flmposition);
                wheeldifference = flmdifference;
            } else if (wheel.equals(Wheel.frm)) {
                double frmposition = frontrightmotor.getCurrentPosition();
                frmdifference = Math.abs(frmtargetposition) - Math.abs(frmposition);
                wheeldifference = frmdifference;
            } else if (wheel.equals(Wheel.blm)) {
                double blmposition = backleftmotor.getCurrentPosition();
                blmdifference = Math.abs(blmtargetposition) - Math.abs(blmposition);
                wheeldifference = blmdifference;
            } else if (wheel.equals(Wheel.brm)) {
                double brmposition = backrightmotor.getCurrentPosition();
                brmdifference = Math.abs(brmtargetposition) - Math.abs(brmposition);
                wheeldifference = brmdifference;
            }
            runWithoutEncoders();


            rotationalheading = degree - get360heading();
            if (rotationalheading < 0) rotationalheading += 360;

            if (wheeldifference < 0) {
                rotationalheading += 180;
                if (rotationalheading >= 360) rotationalheading -= 360;
            }

            radiandirection = Math.toRadians(rotationalheading);

            //double speed = Math.sqrt(.005 * Math.abs(wheeldifference)) + Math.abs(Math.sin(radiandirection) * .1);

            //double speed = inputspeed;

            flmspeed = ((Math.cos(radiandirection) - Math.sin(radiandirection)));
            frmspeed = ((-Math.cos(radiandirection) - Math.sin(radiandirection)));
            blmspeed = ((Math.cos(radiandirection) + Math.sin(radiandirection)));
            brmspeed = ((-Math.cos(radiandirection) + Math.sin(radiandirection)));

            //flmspeed = Math.sqrt(.005 * Math.abs(flmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            //frmspeed = Math.sqrt(.005 * Math.abs(frmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            //blmspeed = Math.sqrt(.005 * Math.abs(blmdifference)) + Math.abs(Math.sin(radiandirection) * .1);
            //brmspeed = Math.sqrt(.005 * Math.abs(brmdifference)) + Math.abs(Math.sin(radiandirection) * .1);



            radheading = Math.toRadians(get360heading());
            radtargetheading = Math.toRadians(targetheading);
            double correction = 0;

            //Our attack allows us to tune how aggressively our robot turns. A lower attach has a
            //faster turn speed.
            double attack = (Math.PI/2);

            //We calculate the distances for what would be a left and what would be a right turn, and
            //we choose the fastest turn path to the ideal final heading.
            radleftdistance = radtargetheading - radheading;
            if (radleftdistance < 0) radleftdistance += (2*(Math.PI));

            radrightdistance = radheading - radtargetheading;
            if (radrightdistance < 0) radrightdistance += (2*(Math.PI));

            //we set the correction
            if (radrightdistance <= radleftdistance) {
                correction = radrightdistance/attack;
            } else {
                correction = -(radleftdistance/attack);
            }

            //and we apply the correction to the motors
            flmspeed = flmspeed + correction;
            frmspeed = frmspeed + correction;
            blmspeed = blmspeed + correction;
            brmspeed = brmspeed + correction;



            double speed = Math.sqrt(.001 * Math.abs(wheeldifference));
            if (speed > inputspeed) {
                speed = inputspeed;
            }
            if(wheeldifference < 0) {
                speed += .05;
            }
            telemetry.addLine("speed " + speed);


            double powerReducer = 1;

            //We create an array of motor power and compare just the extremes
            double [] numbers = {flmspeed, frmspeed, blmspeed, brmspeed};

            Arrays.sort(numbers);

            double min = numbers[0];
            double max = numbers[numbers.length - 1];

            if ((Math.abs(max) > 1) || (Math.abs(min) > 1)) {
                if (Math.abs(max) >= Math.abs(min)) {
                    powerReducer = (1/max);
                } else {
                    powerReducer = (1/Math.abs(min));
                }
            }

            //applying the power reducer
            flmspeed = flmspeed * powerReducer * speed;
            frmspeed = frmspeed * powerReducer * speed;
            blmspeed = blmspeed * powerReducer * speed;
            brmspeed = brmspeed * powerReducer * speed;


            if (wheel.equals(wheel.flm)) {
                if (Math.abs(flmdifference) < 12) {
                    stop = true;
                } else {
                    telemetry.addLine("FLMspeed " + flmspeed);
                    //telemetry.addLine("flmposition " + flmposition);
                    telemetry.addLine("FLMdifference " + flmdifference);
                }
            } else if (wheel.equals(wheel.frm)) {
                if (Math.abs(frmdifference) < 12) {
                    stop = true;
                } else {
                    telemetry.addLine("FRMspeed " + frmspeed);
                    //telemetry.addLine("flmposition " + flmposition);
                    telemetry.addLine("FRMdifference " + frmdifference);
                }
            } else if (wheel.equals(wheel.blm)) {
                if (Math.abs(blmdifference) < 12) {
                    stop = true;
                } else {
                    telemetry.addLine("BLMspeed " + blmspeed);
                    //telemetry.addLine("frmposition " + frmposition);
                    telemetry.addLine("BLMdifference " + blmdifference);
                }
            } else if (wheel.equals(wheel.brm)) {
                if (Math.abs(brmdifference) < 12) {
                    stop = true;
                } else {
                    telemetry.addLine("BRMspeed " + brmspeed);
                    //telemetry.addLine("brmposition " + brmposition);
                    telemetry.addLine("BRMdifference " + brmdifference);
                }
            }

            frontleftmotor.setPower(flmspeed);
            frontrightmotor.setPower(frmspeed);
            backleftmotor.setPower(blmspeed);
            backrightmotor.setPower(brmspeed);

            telemetry.update();
        }
        telemetry.addLine("done");
        telemetry.update();
        stopDrive();
    }

    public void encoderdiaggyrodrivev4 (double speed, int degree, int targetheading, int target) {
        double distance = 0;
        while (opModeIsActive() && (target >= distance)) {
            gyroDiagDrivev2(speed, degree, targetheading);
            distance = encoderdistancetravelledwheel(Wheel.flm);
            if (distance > (target*.8)) {
                speed = ((target-distance)/1000) + .2;
            }
            telemetry.addData("target: ", target);
            telemetry.addData("distance: ", distance);
            telemetry.addData("speed: ", speed);
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

}
