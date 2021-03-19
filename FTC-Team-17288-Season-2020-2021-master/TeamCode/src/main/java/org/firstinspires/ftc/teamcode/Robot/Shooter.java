package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    Telemetry telemetry;
    LinearOpMode opMode;


    public Shooter(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    public double shootValue = -1;
    private double bottom = -0.2;
    private double middle = -0.5;
    private double top = -0.8;

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor shooter;

    public void init(HardwareMap hardwareMap) {
        shooter = hardwareMap.dcMotor.get("sMotor");
    }


    public void shoot(double power){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(power);
//            power = 1;
//        if (power < -0.7)
//            power = -1;
//        shooter.setPower(power);
    }
    public void shootBack(double power){
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(-power);
//            power = 1;
//        if (power < -0.7)
//            power = -1;
//        shooter.setPower(power);
    }

    public void shootStop(){
        shooter.setPower(0);
    }
    public void bottom(){
        shooter.setPower(bottom);
    }

    public void middle(){
        shooter.setPower(middle);
    }

    public void top(){
        shooter.setPower(top);
    }


}