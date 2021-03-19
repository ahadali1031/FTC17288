package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo2;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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
import org.firstinspires.ftc.teamcode.Robot.Vision;
@Autonomous(name = "God 11")

public class VisionAuto extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(telemetry, this);
    Shooter shooter = new Shooter(telemetry,this);
    Shooter2 shooter2 = new Shooter2(telemetry,this);
    Intake intake = new Intake(telemetry, this);
    Container container = new Container(telemetry,this);
    WobbleServo wServo = new WobbleServo(telemetry,this);
    WobbleServo2 wServo2 = new WobbleServo2(telemetry,this);
    WobbleGoal wMotor = new WobbleGoal(telemetry,this);
    IntakeServo iServo = new IntakeServo(telemetry,this);
    ColorTest cSensor = new ColorTest(telemetry,this);
    Vision camera = new Vision();

    public double stopValue = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter2.init(hardwareMap);
        intake.init(hardwareMap);
        container.init(hardwareMap);
        wServo.init(hardwareMap);
        wServo2.init(hardwareMap);
        wMotor.init(hardwareMap);
        cSensor.init(hardwareMap);
        //camera.init(hardwareMap);

        waitForStart();
        runtime.reset();

        int ringCount = camera.getRingCount(hardwareMap);

        telemetry.addData("Ring Count", ringCount);
        telemetry.update();

        if(ringCount == 0) {
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(53, 0.6);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            drivetrain.driveStraight(40,0.6);
            sleep(250);
            drivetrain.turnWithGyro(45);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.driveStraightBack(15,0.6);
            sleep(250);
            wServo.closed();
            wServo2.closed();
            sleep(250);
            drivetrain.driveStraight(15,0.6);
            sleep(250);
            drivetrain.turnWithGyro(45);
            sleep(250);
            drivetrain.turnWithGyro(0);
            sleep(250);
            drivetrain.driveStraightBack(37,0.6);
            sleep(500);
            drivetrain.StraightStrafeLeft(8,0.5);
            wServo.open();
            wServo2.open();
            sleep(250);
            drivetrain.driveStraight(3,0.4);
            sleep(250);
            drivetrain.StraightStrafeRight(33 ,0.5);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(170);
            sleep(250);
            drivetrain.turnWithGyro(179);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.75);
            shooter2.shoot(0.75);
            sleep(2000);
            container.intake(0.8);
            sleep(2500);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            drivetrain.driveStraight(10,0.5);
            sleep(500);
            drivetrain.turnWithGyro(179);


            /*
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(49, 0.6);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            drivetrain.driveStraight(7,0.4);
            sleep(250);
            drivetrain.StraightStrafeRight(34 ,0.5);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(174);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.74);
            shooter2.shoot(0.74);
            sleep(2000);
            container.intake(0.8);
            sleep(2500);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            drivetrain.strafe(Drivetrain.Turn.LEFT,7,0.5);
            sleep(250);
            drivetrain.driveStraightBack(17,0.6);
            sleep(250);
            wMotor.up(0.05);
            sleep(250);
            wServo2.closed();
            wServo.closed();
            sleep(1000);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.FIRST);
            sleep(250);
            drivetrain.driveStraight(50,0.6);
            sleep(250);
            drivetrain.StraightStrafeRight(48,0.5);
            sleep(250);
            drivetrain.driveStraight(3,0.6);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            sleep(500);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.SECOND);
            sleep(250);
            wMotor.up(0.05);
            wServo.closed();
            wServo2.closed();
            sleep(500);
            drivetrain.StraightStrafeLeft(10,0.5);
            sleep(250);
            drivetrain.driveStraightBack(8,0.5);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.FIRST);

             */



        }
        else if (ringCount == 1){
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(80, 0.6);
            sleep(250);
            drivetrain.StraightStrafeRight(24,0.5);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            wMotor.wobbleGoalMovement(WobbleGoal.Position.SECOND);
            sleep(150);
            drivetrain.StraightStrafeLeft(15,0.5);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            drivetrain.driveStraight(63,0.6);
            sleep(250);
            drivetrain.turnWithGyro(50);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.driveStraightBack(13,0.6);
            sleep(500);
            wServo.closed();
            wServo2.closed();
            sleep(500);
            drivetrain.driveStraight(16,0.6);
            sleep(250);
            drivetrain.turnWithGyro(45);
            sleep(250);
            drivetrain.turnWithGyro(0);
            sleep(250);
            drivetrain.driveStraightBack(63,0.6);
            sleep(250);
            drivetrain.StraightStrafeRight(10,0.5);
            sleep(250);
            wServo.open();
            wServo2.open();
            sleep(250);
            drivetrain.driveStraight(22,0.4);
            sleep(250);
            drivetrain.StraightStrafeRight(10,0.5);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(170);
            sleep(250);
            drivetrain.turnWithGyro(179);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.74);
            shooter2.shoot(0.74);
            sleep(2000);
            container.intake(0.8);
            sleep(2500);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            drivetrain.driveStraight(10,0.5);
            sleep(500);
            drivetrain.turnWithGyro(179);
            /*
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(77, 0.7);
            sleep(250);
            drivetrain.StraightStrafeRight(20,0.5);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            sleep(500);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.SECOND);
            sleep(250);
            wMotor.up(0.05);
            wServo.closed();
            wServo2.closed();
            sleep(500);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.FIRST);
            sleep(250);
            drivetrain.driveStraight(28,0.5);
            sleep(250);
            drivetrain.StraightStrafeRight(15,0.5);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(174);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.75);
            shooter2.shoot(0.75);
            sleep(2500);
            container.intake(0.8);
            sleep(3000);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            sleep(250);
            drivetrain.driveStraight(10,0.5);
            sleep(250);

             */

        }
        else if (ringCount == 4){
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(96, 0.7);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            drivetrain.driveStraight(91,0.6);
            sleep(250);
            drivetrain.turnWithGyro(50);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.driveStraightBack(15,0.6);
            sleep(500);
            wServo.closed();
            wServo2.closed();
            sleep(250);
            drivetrain.driveStraight(21,0.6);
            sleep(250);
            wMotor.up(0.4);
            sleep(500);
            drivetrain.turnWithGyro(45);
            sleep(250);
            drivetrain.turnWithGyro(0);
            sleep(250);
            wMotor.stop();
            drivetrain.driveStraightBack(89,0.6);
            sleep(250);
            drivetrain.StraightStrafeLeft(5,0.5);
            sleep(150);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            drivetrain.turnWithGyro(0);
            sleep(250);
            drivetrain.driveStraight(5,0.4);
            sleep(250);
            drivetrain.StraightStrafeRight(33,0.5);
            sleep(250);
            drivetrain.driveStraight(40,0.4);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(170);
            sleep(250);
            drivetrain.turnWithGyro(179);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.74);
            shooter2.shoot(0.74);
            sleep(2000);
            container.intake(0.8);
            sleep(2500);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            drivetrain.driveStraight(10,0.5);
            sleep(250);
            drivetrain.turnWithGyro(179);

            /*
            telemetry.addData("In the if statement", ringCount);
            telemetry.update();
            wServo.closed();
            wServo2.closed();
            telemetry.addData("Servo Closed", ringCount);
            telemetry.update();
            drivetrain.StraightStrafeLeft(7, 0.5);
            telemetry.addData("Drove", ringCount);
            telemetry.update();
            sleep(250);
            drivetrain.driveStraightBack(96, 0.7);
            sleep(250);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(250);
            wServo.open();
            wServo2.open();
            sleep(500);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.SECOND);
            sleep(250);
            wMotor.up(0.05);
            wServo.closed();
            wServo2.closed();
            sleep(500);
            wMotor.wobbleGoalMovement(WobbleGoal.Position.FIRST);
            sleep(250);
            drivetrain.driveStraight(48,0.7);
            sleep(250);
            drivetrain.StraightStrafeRight(40,0.5);
            sleep(250);
            drivetrain.turnWithGyro(90);
            sleep(250);
            drivetrain.turnWithGyro(174);
            sleep(250);
            drivetrain.stopDriving();
            shooter.shoot(0.74);
            shooter2.shoot(0.74);
            sleep(2500);
            container.intake(0.8);
            sleep(3000);
            shooter.shootStop();
            shooter2.shootStop();
            container.stopIntake();
            sleep(250);
            drivetrain.driveStraight(10,0.5);
            sleep(250);

             */
        }
        else{

        }


    }
}

