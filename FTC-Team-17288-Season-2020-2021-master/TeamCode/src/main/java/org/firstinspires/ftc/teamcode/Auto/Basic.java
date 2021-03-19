package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Robot.ColorTest;
import org.firstinspires.ftc.teamcode.Robot.Container;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeServo;
import org.firstinspires.ftc.teamcode.Robot.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Shooter2;
import org.firstinspires.ftc.teamcode.Robot.WobbleGoal;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;

@Autonomous(name = "Safe")

public class Basic extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(telemetry, this);
    Shooter shooter = new Shooter(telemetry,this);
    Shooter2 shooter2 = new Shooter2(telemetry,this);
    Intake intake = new Intake(telemetry, this);
    Container container = new Container(telemetry,this);
    WobbleServo wServo = new WobbleServo(telemetry,this);
    WobbleGoal wMotor = new WobbleGoal(telemetry,this);
    IntakeServo iServo = new IntakeServo(telemetry,this);
    ColorTest cSensor = new ColorTest(telemetry,this);

    public double stopValue = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter2.init(hardwareMap);
        intake.init(hardwareMap);
        container.init(hardwareMap);
        wServo.init(hardwareMap);
        wMotor.init(hardwareMap);
        cSensor.init(hardwareMap);

        waitForStart();
        runtime.reset();

            wServo.closed();
            drivetrain.StraightStrafeRight(9,0.45);
            sleep(200);
            drivetrain.driveStraight(55,0.45);
            sleep(250);
            drivetrain.turnWithGyro(0);
            sleep(500);
            shooter.shoot(0.85);
            shooter2.shoot(0.85);
            sleep(2500);
            container.intake(0.5);
            sleep(3000);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            sleep(250);
            drivetrain.driveStraight(10,0.5);
            sleep(500);
            drivetrain.turnWithGyro(0);
            sleep(500);


        }
    }