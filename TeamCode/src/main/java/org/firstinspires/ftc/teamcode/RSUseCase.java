package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RSUseCase extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    DcMotor LF = null;
    DcMotor RF = null;
    DcMotor LB = null;
    DcMotor RB = null;
    RealSense DepthSensor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status","Initialized", "Tester");
        telemetry.update();
        initial();
        RealSense a = new RealSense(hardwareMap.appContext);
        DepthSensor = a;
        Thread t = new Thread(a);
        t.start();
        drive();
        while(true){
            if (a.getDepth() < 0.5){
                pause();
                runtime.log("Stopped");
                break;
            }
        }
    }
    private void initial(){
        LF = hardwareMap.dcMotor.get("LeftFront");
        RF = hardwareMap.dcMotor.get("RightFront");
        LB = hardwareMap.dcMotor.get("LeftBack");
        RB = hardwareMap.dcMotor.get("RightBack");
    }
    private void drive(){
        LF.setPower(0.2);
        RF.setPower(0.2);
        LB.setPower(0.2);
        RB.setPower(0.2);
    }
    private void pause(){
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }
}
