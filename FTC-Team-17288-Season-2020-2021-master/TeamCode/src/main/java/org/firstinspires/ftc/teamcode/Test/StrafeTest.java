package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot.ColorTest;
import org.firstinspires.ftc.teamcode.Robot.Container;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeServo;
import org.firstinspires.ftc.teamcode.Robot.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Shooter2;
import org.firstinspires.ftc.teamcode.Robot.WobbleGoal;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;
@Disabled
@Autonomous
public class StrafeTest extends LinearOpMode {
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

//        while(opModeIsActive()){
//
//            telemetry.addData("ImuAngle:", drivetrain.imuOutputs());
//            telemetry.update();
//        }
        wMotor.wobbleGoalMovement(WobbleGoal.Position.SECOND);
        sleep(5000);
        wServo.open();
        sleep(1000);
        wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
        sleep(1000);
        wServo.closed();
        sleep(1000);
        wMotor.wobbleGoalMovement(WobbleGoal.Position.FIRST);
    };
}
