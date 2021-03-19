package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    LinearOpMode opMode;


    public Intake(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor intake;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("iMotor");
    }

    public void intake(double power){
        if (power > 0.7)
            power = 1;
        if (power < -0.7)
            power = -1;
        intake.setPower(power);
    }

    public void outtake(double power){
        intake.setPower(-power);
    }

    public void stopIntake(){
        intake.setPower(0);
    }

    public void setIntakeMotor(double value){
        intake.setPower(value);
    }
}
