package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Shooter;

import java.util.ServiceLoader;

import static org.firstinspires.ftc.teamcode.Robot.Drivetrain.Turn.LEFT;
import static org.firstinspires.ftc.teamcode.Robot.Drivetrain.Turn.RIGHT;
@Disabled
@TeleOp(name = "Drivetrain Test")

public class DrivetrainTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(telemetry, this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        waitForStart();
        runtime.reset();



//            drivetrain.turnWithGyroCool(0);
//            sleep(1000);
//            drivetrain.turnWithGyroCool(45);
//            sleep(1000);
//            drivetrain.turnWithGyroCool(90);
//            sleep(1000);
//            drivetrain.turnWithGyroCool(135);
//            sleep(1000);
//        drivetrain.turnWithGyroCool(180);
//        sleep(1000);
//        drivetrain.turnWithGyroCool(225);
//        sleep(1000);
//        drivetrain.turnWithGyroCool(270);
//        sleep(1000);
//        drivetrain.turnWithGyroCool(315);
//        sleep(1000);
//        drivetrain.turnWithGyroCool(360);
//        sleep(1000);







    }

    
}

