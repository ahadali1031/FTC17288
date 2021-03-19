package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleServo {
    Telemetry telemetry;
    LinearOpMode opMode;


    public WobbleServo(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    private ElapsedTime runtime = new ElapsedTime();

    public Servo wobble;

    public void init(HardwareMap hardwareMap) {
        wobble = hardwareMap.servo.get("wServo");
    }

    public void maxPosition(){
        wobble.setPosition(Servo.MAX_POSITION);
    }

    public void minPosition(){
        wobble.setPosition(Servo.MIN_POSITION);
    }

    public void setPosition(double value){
        wobble.setPosition(value);
    }

    public void closed(){
        wobble.setPosition(0.47);
    }

    public void open(){
        wobble.setPosition(0.95);
    }
}
