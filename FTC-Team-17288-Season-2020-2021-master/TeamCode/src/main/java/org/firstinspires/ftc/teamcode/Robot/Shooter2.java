package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter2 {
    Telemetry telemetry;
    LinearOpMode opMode;


    public Shooter2(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    public double shootValue = -1;
    private double bottom = -0.2;
    private double middle = -0.5;
    private double top = -0.8;

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor shooter2;

    public void init(HardwareMap hardwareMap) {
        shooter2 = hardwareMap.dcMotor.get("s2Motor");
    }


    public void shoot(double power){
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setPower(power);
//        if (power > 0.7)
//            power = 1;
//        if (power < -0.7)
//            power = -1;
//        shooter2.setPower(-power);
    }

    public void shootBack(double power){
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setPower(-power);
//        if (power > 0.7)
//            power = 1;
//        if (power < -0.7)
//            power = -1;
//        shooter2.setPower(-power);
    }

    public void shootStop(){
        shooter2.setPower(0);
    }
    public void bottom(){
        shooter2.setPower(bottom);
    }

    public void middle(){
        shooter2.setPower(middle);
    }

    public void top(){
        shooter2.setPower(top);
    }


}