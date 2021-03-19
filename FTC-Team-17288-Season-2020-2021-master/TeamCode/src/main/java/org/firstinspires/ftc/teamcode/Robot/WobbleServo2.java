package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleServo2 {
    Telemetry telemetry;
    LinearOpMode opMode;


    public WobbleServo2(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    private ElapsedTime runtime = new ElapsedTime();

    public Servo wobble2;

    public void init(HardwareMap hardwareMap) {
        wobble2 = hardwareMap.servo.get("wServo2");
    }

    public void open() {
        wobble2.setPosition(0);
    }
    public void closed(){
        wobble2.setPosition(0.59);
    }
}
