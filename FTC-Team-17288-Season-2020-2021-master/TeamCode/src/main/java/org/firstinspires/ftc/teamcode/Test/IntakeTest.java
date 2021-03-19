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

@TeleOp(name = "Badri sucks lmao")
@Disabled
public class IntakeTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Intake intakeMot = new Intake(telemetry, this);


    @Override
    public void runOpMode() throws InterruptedException {
        intakeMot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {



        }


    }

}

