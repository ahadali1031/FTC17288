 package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Drivetrain {

    Telemetry telemetry;
    LinearOpMode opMode;

    public Drivetrain(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    public enum Direction {
        FORWARD, BACKWARD
    }

    public enum Turn {
        LEFT, RIGHT
    }

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    // Gyroscope //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //double kp = 0.027; // test and edit later on //
    double kp = 0.018;
    double targetAngle = 0;
    double modularAngle;
    double error;
    double Power;

    private final double COUNTS_PER_MOTOR_REV = 1120 / 2.5;    // Andy Mark Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    private final double ONE_ROTATION_INCHES = WHEEL_DIAMETER_INCHES * Math.PI; // circumferences
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / ONE_ROTATION_INCHES;

    private double SCALING_RATIO = 0.95;
    private double SLOW_SCALING_RATIO = 0.6;


    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");

        // Gyroscope //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;    // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;    // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;
        return input;
    }

    // Simple drivetrain methods //
    public void drive(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void brake() {
        frontRight.setPower(-0.04);
        frontLeft.setPower(-0.04);
        backRight.setPower(-0.04);
        backLeft.setPower(-0.04);
    }

    public void stopDriving() {
        drive(0);
    }

    public void turnRight(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    public void turnLeft(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void strafeLeft(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(-power);
    }

    public void strafeRight(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    public void diagonalUpRight(double power) {
        frontLeft.setPower(power);
        backRight.setPower(power);
    }

    public void diagonalUpLeft(double power) {
        frontRight.setPower(power);
        backLeft.setPower(power);
    }

    // complex drivetrain methods //
    public void turnWithEncoder(Turn turn, double distance, double power) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (turn == Turn.RIGHT) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
        } else if (turn == Turn.LEFT) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {

        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

//    public void turnWithGyro(double Angle) {
//
//        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        targetAngle = Angle;
//
//        if (targetAngle % 180 == 0 || targetAngle != 0)
//            modularAngle = 180;
//        else
//            modularAngle = (targetAngle) % 180;
//
//        while (Math.abs(targetAngle - angles.firstAngle) > 3.5 && opMode.opModeIsActive()) {
//
//            error = targetAngle - angles.firstAngle;
//            Power = (kp) * (error);
//
//            frontLeft.setPower(-Power);
//            frontRight.setPower(Power);
//            backLeft.setPower(-Power);
//            backRight.setPower(Power);
//
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            telemetry.addData("error: ", error);
//            telemetry.addData("target: ", targetAngle);
//            telemetry.addData("angle: ", angles.firstAngle);
//            telemetry.addData("modular angle", modularAngle);
//            telemetry.update();
//
//        }
//    }

    public void turnWithGyro(double Angle) {

        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        targetAngle = Angle;

        if (targetAngle % 180 == 0 || targetAngle != 0)
            modularAngle = 180;
        else
            modularAngle = (targetAngle) % 180;

        error = inputModulus(targetAngle-angles.firstAngle,-180,180);


        while (Math.abs(error) > 3.5 && opMode.opModeIsActive()) {

            error = inputModulus(targetAngle-angles.firstAngle,-180,180);
            Power = (kp) * (error);

            frontLeft.setPower(-Power);
            frontRight.setPower(Power);
            backLeft.setPower(-Power);
            backRight.setPower(Power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("error: ", error);
            telemetry.addData("target: ", targetAngle);
            telemetry.addData("angle: ", angles.firstAngle);
            telemetry.addData("modular angle", modularAngle);
            telemetry.update();

        }
    }



        //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontLeft.setTargetPosition(1000);
        //frontLeft.setPower(.5);
        //while(frontLeft.isBusy())
        //{}




    public void strafe(Turn direction, double distance, double power) {
        int Target;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Target = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Turn.LEFT) {
            frontLeft.setTargetPosition(-Target);
            frontRight.setTargetPosition(Target);
            backLeft.setTargetPosition(Target);
            backRight.setTargetPosition(-Target);
        } else if (direction == Turn.RIGHT) {
            frontLeft.setTargetPosition(Target);
            frontRight.setTargetPosition(-Target);
            backLeft.setTargetPosition(-Target);
            backRight.setTargetPosition(Target);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {
        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveAdjusted(Direction direction, double distance, double powerLeft,double powerRight) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Direction.FORWARD) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
        } else if (direction == Direction.BACKWARD) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        frontLeft.setPower(powerLeft);
        backLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backRight.setPower(powerRight);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {

        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveStraight (int distance,double power) throws InterruptedException {
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        distance = (int) (distance * COUNTS_PER_INCH);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;

        double startPosition = frontLeft.getCurrentPosition();

        while (frontLeft.getCurrentPosition() < distance + startPosition) {
            double zAccumulated = angles.firstAngle;

            frontLeftSpeed = power - (zAccumulated - target) / 100;
            frontRightSpeed = power + (zAccumulated - target) / 100;
            backLeftSpeed = power - (zAccumulated - target) / 100;
            backRightSpeed = power + (zAccumulated - target) / 100;

            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
            backLeftSpeed = Range.clip(backLeftSpeed, -1, 1);
            backRightSpeed = Range.clip(backRightSpeed, -1, 1);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            telemetry.addData("1. FrontLeft", frontLeft.getPower());
            telemetry.addData("2. FrontRight", frontRight.getPower());
            telemetry.addData("3. BackLeft", backLeft.getPower());
            telemetry.addData("4. BackRight", backRight.getPower());
            telemetry.addData("Distance to go:", distance + startPosition - frontLeft.getCurrentPosition());
            telemetry.addData("angle",angles.firstAngle);
            telemetry.update();

            opMode.sleep(1);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        opMode.sleep(1);
    }

    public void StraightStrafeRight (int distance,double power) throws InterruptedException {
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        distance = (int) (distance * COUNTS_PER_INCH);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;

        double startPosition = frontLeft.getCurrentPosition();

        while (frontLeft.getCurrentPosition() < distance + startPosition) {
            double zAccumulated = angles.firstAngle;

            frontLeftSpeed = power + (target-zAccumulated) / 100;
            frontRightSpeed = -power - (target-zAccumulated) / 100;
            backLeftSpeed = -power - (target-zAccumulated) / 100;
            backRightSpeed = power +  (target-zAccumulated) / 100;

            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
            backLeftSpeed = Range.clip(backLeftSpeed, -1, 1);
            backRightSpeed = Range.clip(backRightSpeed, -1, 1);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("1. FrontLeft", frontLeft.getPower());
            telemetry.addData("2. FrontRight", frontRight.getPower());
            telemetry.addData("3. BackLeft", backLeft.getPower());
            telemetry.addData("4. BackRight", backRight.getPower());
            telemetry.addData("Distance to go:", distance + startPosition - frontLeft.getCurrentPosition());
            telemetry.addData("angle",angles.firstAngle);
            telemetry.update();

            opMode.sleep(1);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        opMode.sleep(1);
    }

    public void StraightStrafeLeft (int distance,double power) throws InterruptedException {
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        distance = (int) (distance * COUNTS_PER_INCH);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;

        double startPosition = frontRight.getCurrentPosition();

        while (frontRight.getCurrentPosition() < distance + startPosition) {
            double zAccumulated = angles.firstAngle;

            frontLeftSpeed = -power - (target-zAccumulated) / 100;
            frontRightSpeed = power + (target-zAccumulated) / 100;
            backLeftSpeed = power - (target-zAccumulated) / 100;
            backRightSpeed = -power + (target-zAccumulated) / 100;

            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
            backLeftSpeed = Range.clip(backLeftSpeed, -1, 1);
            backRightSpeed = Range.clip(backRightSpeed, -1, 1);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("1. FrontLeft", frontLeft.getPower());
            telemetry.addData("2. FrontRight", frontRight.getPower());
            telemetry.addData("3. BackLeft", backLeft.getPower());
            telemetry.addData("4. BackRight", backRight.getPower());
            telemetry.addData("Distance to go:", distance + startPosition - frontRight.getCurrentPosition());
            telemetry.addData("angle",zAccumulated);
            telemetry.addData("target",target);
            telemetry.addData("error:",target-zAccumulated);
            telemetry.update();

            opMode.sleep(1);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        opMode.sleep(1);
    }

    public void StraightStrafeLeftInput (int distance,double power,double angle) throws InterruptedException {
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        distance = (int) (distance * COUNTS_PER_INCH);

        double target = angle;

        double startPosition = frontRight.getCurrentPosition();

        while (frontRight.getCurrentPosition() < distance + startPosition) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double zAccumulated = angles.firstAngle;

            frontLeftSpeed = -power - (target-zAccumulated) / 100;
            frontRightSpeed = power + (target-zAccumulated) / 100;
            backLeftSpeed = power - (target-zAccumulated) / 100;
            backRightSpeed = -power + (target-zAccumulated) / 100;

            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
            backLeftSpeed = Range.clip(backLeftSpeed, -1, 1);
            backRightSpeed = Range.clip(backRightSpeed, -1, 1);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("1. FrontLeft", frontLeft.getPower());
            telemetry.addData("2. FrontRight", frontRight.getPower());
            telemetry.addData("3. BackLeft", backLeft.getPower());
            telemetry.addData("4. BackRight", backRight.getPower());
            telemetry.addData("Distance to go:", distance + startPosition - frontRight.getCurrentPosition());
            telemetry.addData("angle",zAccumulated);
            telemetry.addData("target",target);
            telemetry.update();

            opMode.sleep(1);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);


        opMode.sleep(1);
    }

    public void driveStraightBack (int distance,double power) throws InterruptedException {
        double frontLeftSpeed;
        double frontRightSpeed;
        double backLeftSpeed;
        double backRightSpeed;
        distance = -1* (int) (distance * COUNTS_PER_INCH);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double target = angles.firstAngle;

        double startPosition = frontLeft.getCurrentPosition();

        while (frontLeft.getCurrentPosition() > distance + startPosition) {
            double zAccumulated = angles.firstAngle;

            frontLeftSpeed = -power - (target-zAccumulated) / 100;
            frontRightSpeed = -power + (target-zAccumulated) / 100;
            backLeftSpeed = -power - (target-zAccumulated) / 100;
            backRightSpeed = -power + (target-zAccumulated) / 100;

            frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
            frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
            backLeftSpeed = Range.clip(backLeftSpeed, -1, 1);
            backRightSpeed = Range.clip(backRightSpeed, -1, 1);

            frontLeft.setPower(frontLeftSpeed);
            frontRight.setPower(frontRightSpeed);
            backLeft.setPower(backLeftSpeed);
            backRight.setPower(backRightSpeed);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("1. FrontLeft", frontLeft.getPower());
            telemetry.addData("2. FrontRight", frontRight.getPower());
            telemetry.addData("3. BackLeft", backLeft.getPower());
            telemetry.addData("4. BackRight", backRight.getPower());
            telemetry.addData("Distance to go:", distance + startPosition - frontLeft.getCurrentPosition());
            telemetry.addData("angle",angles.firstAngle);
            telemetry.update();

            opMode.sleep(1);


        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        opMode.sleep(1);
    }

    public void gyro_encoder_drive (Direction direction, double distance, double power, double angle) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Direction.FORWARD) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
        } else if (direction == Direction.BACKWARD) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        targetAngle = angle;

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        drive(power);



            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {
                if (targetAngle % 180 == 0 || targetAngle != 0)
                    modularAngle = 180;
                else
                    modularAngle = (targetAngle) % 180;

                if (Math.abs(targetAngle - angles.firstAngle) > 5 && opMode.opModeIsActive()) {

                    error = targetAngle - angles.firstAngle;
                    Power = (kp) * (error);

                    frontLeft.setPower(-Power);
                    frontRight.setPower(Power);
                    backLeft.setPower(-Power);
                    backRight.setPower(Power);

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    telemetry.addData("error: ", error);
                    telemetry.addData("target: ", targetAngle);
                    telemetry.addData("angle: ", angles.firstAngle);
                    telemetry.addData("modular angle", modularAngle);
                    telemetry.update();
            }

            stopDriving();

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void drive(Direction direction, double distance, double power) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Direction.FORWARD) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
        } else if (direction == Direction.BACKWARD) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        drive(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {

        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double imuOutputs(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    // Telo-Op //
    public void ArcadeDrive(double linear, double rotation, double strafe) {

        frontLeft.setPower((rotation - linear + strafe) * SCALING_RATIO);
        backLeft.setPower((rotation - linear - strafe) * SCALING_RATIO);
        frontRight.setPower((-rotation - linear - strafe) * SCALING_RATIO);
        backRight.setPower((-rotation - linear + strafe) * SCALING_RATIO);
    }
    public void ArcadeDriveSlow(double linear, double rotation, double strafe) {
        frontLeft.setPower((rotation - linear + strafe) * SLOW_SCALING_RATIO);
        backLeft.setPower((rotation - linear - strafe) * SLOW_SCALING_RATIO);
        frontRight.setPower((-rotation - linear - strafe) * SLOW_SCALING_RATIO);
        backRight.setPower((-rotation - linear + strafe) * SLOW_SCALING_RATIO);
    }

    public void TankDrive(double left, double right, double strafe) {
        frontLeft.setPower((left + strafe) * SCALING_RATIO);
        backLeft.setPower((left - strafe) * SCALING_RATIO);
        frontRight.setPower((right - strafe) * SCALING_RATIO);
        backRight.setPower((right + strafe) * SCALING_RATIO);
    }

    public void TankDriveSlow(double left, double right, double strafe) {
        frontLeft.setPower((left + strafe) * SLOW_SCALING_RATIO);
        backLeft.setPower((left - strafe) * SLOW_SCALING_RATIO);
        frontRight.setPower((right - strafe) * SLOW_SCALING_RATIO);
        backRight.setPower((right + strafe) * SLOW_SCALING_RATIO);


    }
}