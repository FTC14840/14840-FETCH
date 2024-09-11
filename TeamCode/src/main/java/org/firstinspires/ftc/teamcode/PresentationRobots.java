package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Configuration File
 * Control Hub:
 * Motor Port 00: leftFront
 * Motor Port 01: leftBack
 * Motor Port 02: rightBack
 * Motor Port 03: rightFront
 * Servo Port 00: claw
 */

//@Disabled
@TeleOp(name="TicTacToe")

public class PresentationRobots extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightBack, rightFront;
    private double leftFrontPower, leftBackPower, rightBackPower, rightFrontPower;
    private double driveAxial, driveLateral, driveYaw;
    private double driveZeroPower = 0.0;
    private double maxPower;

    private boolean speedButton = false;
    private boolean directionButton = false;
    private double speed = 0.50;
    private double direction = 1.0;
    private static final double LATERAL_SENSITIVITY = 1.0;
    private static final double YAW_SENSITIVITY = 1.0;

    private Servo claw;
    private static final double clawOpen = 0.58;
    private static final double clawClose = 0.51;

    @Override
    public void runOpMode() {
        initTelemetry();
        initHardware();
        while (!isStarted()) {
            initFinishedTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            manualDrive();
            moveRobot();
            driveTelemetry();
        }
    }

    public void initHardware() {
        initDrivetrain();
        initClaw();
    }

    public void initDrivetrain() {
        driveHardwareMap();
        driveDirection();
        drivetrainBrake();
        stopDriving();
        stopAndResetEncoder();
        RunWithoutEncoder();
    }

    public void driveHardwareMap() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    }

    public void driveDirection() {
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void drivetrainBrake() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stopDriving() {
        leftFront.setPower(driveZeroPower);
        leftBack.setPower(driveZeroPower);
        rightBack.setPower(driveZeroPower);
        rightFront.setPower(driveZeroPower);
    }

    public void stopAndResetEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunWithoutEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initClaw() {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(clawOpen);
    }

    public void manualDrive() {
        setAxial(-gamepad1.left_stick_y); // Forward on the y-stick is negative: inverse. Adding the negative (-) fixes that.
        setLateral(gamepad1.left_stick_x);
        setYaw(gamepad1.right_stick_x);

        if (gamepad1.a) {
            claw.setPosition(clawOpen);
        }

        if (gamepad1.b) {
            claw.setPosition(clawClose);
        }
    }

    public void setAxial (double axial) {
        driveAxial = Range.clip(axial, -1.0, 1.0);
    }

    public void setLateral (double lateral) {
        driveLateral = Range.clip(lateral, -1.0, 1.0);
    }

    public void setYaw (double yaw) {
        driveYaw = Range.clip(yaw, -1.0, 1.0);
    }

    public void moveRobotVectorAddition() {
        leftFrontPower = speed * (direction * ((driveAxial) + (driveLateral * LATERAL_SENSITIVITY)) + (driveYaw * YAW_SENSITIVITY));
        leftBackPower = speed * (direction * ((driveAxial) - (driveLateral * LATERAL_SENSITIVITY)) + (driveYaw * YAW_SENSITIVITY));
        rightFrontPower = speed * (direction * ((driveAxial) - (driveLateral * LATERAL_SENSITIVITY)) - (driveYaw * YAW_SENSITIVITY));
        rightBackPower = speed * (direction * ((driveAxial) + (driveLateral * LATERAL_SENSITIVITY)) - (driveYaw * YAW_SENSITIVITY));
    }

    public void moveRobotMaxPower() {
        maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
    }

    public void moveRobotSetMaxPower() {
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
            rightFrontPower /= maxPower;
        }
    }

    public void moveRobotSetPower() {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public void moveRobot() {
        moveRobotVectorAddition();
        moveRobotMaxPower();
        moveRobotSetMaxPower();
        moveRobotSetPower();
    }

    public void initTelemetry() {
        telemetry.log().add("Robot Initializing. Do Not Move The Bot...");
        telemetry.update();
    }

    public void initFinishedTelemetry() {
        telemetry.addData("Robot Initialized", "Press Play to Begin");
        telemetry.update();
    }

    public void driveTelemetry() {
        telemetry.addData("Power", "LF: %.2f, LB: %.2f, RB: %.2f, RF: %.2f", leftFront.getPower(), leftBack.getPower(), rightBack.getPower(), rightFront.getPower());
        telemetry.addData("Axes", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        telemetry.update();
    }
}