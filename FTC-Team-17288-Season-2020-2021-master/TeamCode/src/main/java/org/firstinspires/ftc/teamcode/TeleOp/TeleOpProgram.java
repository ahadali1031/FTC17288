package org.firstinspires.ftc.teamcode.TeleOp;

import android.text.Layout;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.ColorTest;
import org.firstinspires.ftc.teamcode.Robot.Container;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeServo;
import org.firstinspires.ftc.teamcode.Robot.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Shooter2;
import org.firstinspires.ftc.teamcode.Robot.WobbleGoal;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;


@TeleOp (name = "Tanvi is a Tank")
public class TeleOpProgram extends LinearOpMode {

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

        boolean Mode = false;
        boolean Speed = true;
        boolean alignment = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Drivetrain //

            // Tank Drive //

            if(gamepad1.right_stick_button && !Mode){
                Mode = true;
                Speed = !Speed;
            }
            else if (!gamepad1.right_stick_button && Mode){
                Mode = false;
            }

            if (Speed) {
                drivetrain.TankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.left_stick_x);
            }
            else{
                drivetrain.TankDriveSlow(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.left_stick_x);
        }
            intake.intake(gamepad2.right_stick_y);
            wMotor.power(gamepad2.right_trigger);
            wMotor.power(-gamepad2.left_trigger);
            //wMotor.up(gamepad2.left_trigger);
            //wMotor.down(gamepad2.right_trigger);
            if (gamepad2.right_bumper){
                shooter.shoot(0.85);
                shooter2.shoot(0.85);
            }
            else if (gamepad2.left_bumper){
                shooter.shootBack(0.85);
                shooter2.shootBack(0.85);
            }
            else{
                shooter.shootStop();
                shooter2.shootStop();
            }

            if (gamepad1.x)
                drivetrain.turnWithGyro(0);






//            while(gamepad2.right_bumper) {
//                intake.intake(1);
//                drivetrain.TankDrive(-gamepad1.left_stick_y,-gamepad1.right_stick_y , gamepad1.left_stick_x);
//                shooter.shoot(-gamepad2.left_stick_y);
//                shooter2.shoot(-gamepad2.left_stick_y);
//                if (!gamepad2.right_bumper)
//                    intake.stopIntake();
//                    break;
//            }
//            while (gamepad2.left_bumper) {
//                intake.outtake( 1);
//                drivetrain.TankDrive(-gamepad1.left_stick_y,-gamepad1.right_stick_y , gamepad1.left_stick_x);
//                shooter.shoot(-gamepad2.left_stick_y);
//                shooter2.shoot(-gamepad2.left_stick_y);
//                if (!gamepad2.left_bumper)
//                    intake.stopIntake();
//                    break;
//            }

            if (gamepad1.right_bumper) {
                container.intake(0.5);
            }
            else if (gamepad1.left_bumper) {
                container.outtake(0.5);
            }
            else
                container.stopIntake();

            if (gamepad2.x){
                wServo.open();

            }

            if (gamepad2.circle){
                wServo.closed();

            }





            // Arcade Drive //
            //drivetrain.ArcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

}
