package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Container {
    Telemetry telemetry;
    LinearOpMode opMode;


    public Container(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    private ElapsedTime runtime = new ElapsedTime();

    public CRServo container;

    public void init(HardwareMap hardwareMap) {
        container = hardwareMap.crservo.get("cServo");
    }

    public void intake(double power){
        container.setPower(-power);
    }

    public void outtake(double power){
        container.setPower( power);
    }

    public void stopIntake(){
        container.setPower(0);
    }

}
