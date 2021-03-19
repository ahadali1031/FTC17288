package org.firstinspires.ftc.teamcode.Test;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.WobbleGoal;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo;
import org.firstinspires.ftc.teamcode.Robot.WobbleServo2;

import java.util.ServiceLoader;

// Use this code to get the tick values associated with each position //
@TeleOp(name = "Wobble Goal Test")
public class WobbleGoalTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    WobbleGoal wobble = new WobbleGoal(telemetry,this);
    WobbleServo wobbleServo = new WobbleServo(telemetry,this);
    WobbleServo2 wobbleServo2 = new WobbleServo2(telemetry,this);
    @Override
    public void runOpMode() throws InterruptedException {
        wobble.init(hardwareMap);
        wobbleServo.init(hardwareMap);
        wobbleServo2.init(hardwareMap);

        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {
//             Wobble Pick Up Code
            sleep(250);
            wobble.wobbleGoalMovement(WobbleGoal.Position.SECOND);
            sleep(200);
            wobble.up(0.05);
            wobbleServo.open();
            wobbleServo2.open();
            sleep(1000);
            wobble.wobbleGoalMovement(WobbleGoal.Position.THIRD);
            sleep(500);
            wobble.up(0.05);
            sleep(250);
            wobbleServo2.closed();
            wobbleServo.closed();
            sleep(1000);
            wobble.wobbleGoalMovement(WobbleGoal.Position.FIRST);
            sleep(250);

            }



        }
    }

