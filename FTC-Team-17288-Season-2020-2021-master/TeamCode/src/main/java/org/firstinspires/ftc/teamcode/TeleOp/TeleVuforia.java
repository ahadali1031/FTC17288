package org.firstinspires.ftc.teamcode.TeleOp;

import android.text.Layout;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robot.ColorTest;
import org.firstinspires.ftc.teamcode.Robot.Container;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Intake;
import org.firstinspires.ftc.teamcode.Robot.IntakeServo;
import org.firstinspires.ftc.teamcode.Robot.Shooter;
import org.firstinspires.ftc.teamcode.Robot.Shooter2;
import org.firstinspires.ftc.teamcode.Robot.WobbleGoal;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo2;
import org.firstinspires.ftc.teamcode.Robot.Vision;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


@TeleOp (name = "Tanvi is a Slow Tank with a Camera")
public class TeleVuforia extends LinearOpMode {

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

        boolean dMode = false;
        boolean dSpeed = true;
        boolean sMode = false;
        boolean sSpeed = true;
        boolean alignment = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Drivetrain //

            // Tank Drive //

            if(gamepad1.right_stick_button && !dMode){
                dMode = true;
                dSpeed = !dSpeed;
            }
            else if (!gamepad1.right_stick_button && dMode){
                dMode = false;
            }


            if (dSpeed) {
                drivetrain.TankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.left_stick_x);
            }
            else{
                drivetrain.TankDriveSlow(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.left_stick_x);
                telemetry.addData("Drivetrain","Slow");
            }

            if(gamepad2.left_stick_button && !sMode){
                sMode = true;
                sSpeed = !sSpeed;
            }
            else if (!gamepad2.left_stick_button && sMode){
                sMode = false;
            }

            if (!sSpeed){
                telemetry.addData("Shooter:","Slow");
            }
            else if (sSpeed){
                telemetry.addData("Shooter:","Fast");
            }
            intake.intake(gamepad2.right_stick_y);
            wMotor.power(gamepad2.right_trigger);
            wMotor.power(-gamepad2.left_trigger);
            //wMotor.up(gamepad2.left_trigger);
            //wMotor.down(gamepad2.right_trigger);
            if (gamepad2.right_bumper && !sSpeed){
                shooter.shoot(0.68);
                shooter2.shoot(0.68);
            }
            else if (gamepad2.right_bumper){
                shooter.shoot(0.75);
                shooter2.shoot(0.75);
            }
            else if (gamepad2.left_bumper){
                shooter.shootBack(0.75);
                shooter2.shootBack(0.75);
            }
            else{
                shooter.shootStop();
                shooter2.shootStop();
            }









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

            if (gamepad2.dpad_right) {
                container.intake(0.8);
            }
            else if (gamepad2.dpad_left) {
                container.outtake(0.8);
            }
            else
                container.stopIntake();

            if (gamepad2.x){
                wServo.open();
                wServo2.open();

            }

            if (gamepad2.circle){
                wServo.closed();
                wServo2.closed();
            }
            if (gamepad1.x)
                drivetrain.turnWithGyro(0);

            if(gamepad1.dpad_left){
                drivetrain.StraightStrafeLeft(5,0.5);
            }
            if(gamepad1.dpad_right){
                drivetrain.StraightStrafeRight(5,0.5);
            }
            telemetry.update();






            // Arcade Drive //
            //drivetrain.ArcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

}
