package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WobbleGoal {

    Telemetry telemetry;
    LinearOpMode opMode;

    public WobbleGoal(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    private DcMotor intake;

    // These are the speeds; change if you want faster or slower movement of motor //
    private double UP_SCALING_RATIO = 0.5;
    private double DOWN_SCALING_RATIO = 0.3;
    private double NO_POWER = 0.0;

    // Use telemetry to get the tick values associated with each position //
    // I assume the initial position is associated with 0 because motor hasn't moved yet //
    private double INITIAL_POSITION = 0.0;
    private double SECOND_POSITION = -250;
    private double THIRD_POSITION = -410;

    public enum Position {
        FIRST, SECOND, THIRD
    }

    public void init (HardwareMap hardwareMap){
        intake = hardwareMap.dcMotor.get("wMotor");

        // If everything moves wrong way on first test, try changing the direction to REVERSE //
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void power(double power){
        intake.setPower(power);
    }
    public void stop(){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(NO_POWER);
    }

    public void up(){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(UP_SCALING_RATIO);
    }

    public void up(double input){
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(UP_SCALING_RATIO*input);
    }

    public void down(){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(-DOWN_SCALING_RATIO);
    }

    public void down(double input){
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(-DOWN_SCALING_RATIO*input);
    }

    public void wobbleGoalMovement(Position position) {
        if (position == Position.FIRST) {
            // If I messed up the logic, try changing the ">" to "<" and the other way around as well //
            if (getTicks() > INITIAL_POSITION) {
                while (getTicks() > INITIAL_POSITION)
                    down();
                stop();
            }
            else if (getTicks() < INITIAL_POSITION) {
                while (getTicks() < INITIAL_POSITION)
                    up();
                stop();
            }
            else
                stop();
        }
        else if (position == Position.SECOND){
            if (getTicks() > SECOND_POSITION) {
                while (getTicks() > SECOND_POSITION)
                    down();
                stop();
            }
            else if (getTicks() < SECOND_POSITION) {
                while (getTicks() < SECOND_POSITION)
                    up();
                stop();
            }
            else
                stop();
        }
        else if (position == Position.THIRD){
            if (getTicks() > THIRD_POSITION) {
                while (getTicks() > THIRD_POSITION)
                    down();
                stop();
            }
            else if (getTicks() < THIRD_POSITION) {
                while (getTicks() < THIRD_POSITION)
                    up();
                stop();
            }
            else
                stop();
        }
        else
            stop();
    }

    public void teleopMovement(boolean up, boolean down){
        if (up)
            up();
        else if (down)
            down();
        else
            stop();
    }

    public int getTicks(){
        return intake.getCurrentPosition();
    }

    public void resetEncoders(){
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}