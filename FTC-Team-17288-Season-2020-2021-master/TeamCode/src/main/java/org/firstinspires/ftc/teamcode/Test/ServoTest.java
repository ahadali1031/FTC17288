package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeServo;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;

@TeleOp(name = "Useless like Badri")
@Disabled
public class ServoTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    WobbleServo wobble = new WobbleServo(telemetry, this);
    IntakeServo servo = new IntakeServo(telemetry,this);

    @Override
    public void runOpMode() throws InterruptedException {
        wobble.init(hardwareMap);
        servo.init(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            for (double i = 0; i <= 1; i+=0.1){
                servo.setPosition(i);
                telemetry.addData("Position",i);
                telemetry.update();
                sleep(2000);
            }
            stop();



        }


    }

}

