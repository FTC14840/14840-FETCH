package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates how to use the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, can
 * recognize colors, and can be trained to detect custom objects. See this website for
 * documentation: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336
 *
 * This sample illustrates how to detect AprilTags, but can be used to detect other types
 * of objects by changing the algorithm. It assumes that the HuskyLens is configured with
 * a name of "huskylens".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "FetchNoPID", group = "Sensor")
//@Disabled
public class FetchCodeNoPID extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    private double MOTOR_POWER = 0.20;
    private double TURN_MOTOR_POWER = 0.10;
    private double SEARCH_MOTOR_POWER = 0.25;
    private double LeftFrontPower = MOTOR_POWER;
    private double LeftRearPower = MOTOR_POWER;
    private double RightFrontPower = MOTOR_POWER;
    private double RightRearPower = MOTOR_POWER;

    private double clawOpenPosition = 1;
    private double clawClosePosition = 0;

    private int lastX = 125;


    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private Servo claw;
    @Override
    public void runOpMode() throws InterruptedException {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightBack");
        leftRear  = hardwareMap.get(DcMotor.class, "leftBack");
        rightRear = hardwareMap.get(DcMotor.class, "rightFront");

        claw = hardwareMap.get(Servo.class, "claw");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(Servo.Direction.FORWARD);



        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);


        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while(opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData(">> ", blocks[i].toString());
                HuskyLens.Block block = blocks[i];
                lastX = block.x;
                if(block.x < 123) {
                    LeftFrontPower = MOTOR_POWER + -TURN_MOTOR_POWER;
                    LeftRearPower = MOTOR_POWER + -TURN_MOTOR_POWER;

                    RightFrontPower = MOTOR_POWER + TURN_MOTOR_POWER;
                    RightRearPower = MOTOR_POWER + TURN_MOTOR_POWER;
                } else if(block.x > 225 ) {

                    LeftFrontPower = MOTOR_POWER + TURN_MOTOR_POWER;
                    LeftRearPower = MOTOR_POWER + TURN_MOTOR_POWER;

                    RightFrontPower = MOTOR_POWER + -TURN_MOTOR_POWER;
                    RightRearPower = MOTOR_POWER + -TURN_MOTOR_POWER;
                } else {
                    LeftFrontPower = MOTOR_POWER;
                    LeftRearPower = MOTOR_POWER;

                    RightFrontPower = MOTOR_POWER;
                    RightRearPower = MOTOR_POWER;
                }

                if(block.width < 80) {
                    leftFront.setPower(LeftFrontPower);
                    rightFront.setPower(RightFrontPower);
                    leftRear.setPower(LeftRearPower);
                    rightRear.setPower(RightRearPower);
                    claw.setPosition(clawOpenPosition);

                } else {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);


                    if(checkIfCaptured()) {
                        capturedTheBall();
                    }
                }


            }
            if(blocks.length == 0) {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);

                claw.setPosition(clawOpenPosition);
                if (checkIfCaptured()) {
                    capturedTheBall();
                } else {
                    boolean found = search();
                    int amtFailed = 0;
                    while (!found) {
                        amtFailed++;
                        found = search();
                        if(amtFailed > 3) {
                            moveForward();
                            sleep(4000);
                            stop_bot();
                            amtFailed = 0;
                        }

                    }
                }


            }
            telemetry.update();



        }

    }
    boolean checkIfCanSee() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if(blocks.length == 0) {
            return false;
        } else {
            return true;
        }
    }
    void moveForward() {
        leftFront.setPower(MOTOR_POWER);
        leftRear.setPower(MOTOR_POWER);
        rightFront.setPower(MOTOR_POWER);
        rightRear.setPower(MOTOR_POWER);
    }
    void moveBackward() {
        leftFront.setPower(-MOTOR_POWER);
        leftRear.setPower(-MOTOR_POWER);
        rightFront.setPower(-MOTOR_POWER);
        rightRear.setPower(-MOTOR_POWER);
    }
    void turnLeft() {
        leftFront.setPower(-MOTOR_POWER);
        leftRear.setPower(-MOTOR_POWER);
        rightFront.setPower(MOTOR_POWER);
        rightRear.setPower(MOTOR_POWER);
    }
    void turnRight() {
        leftFront.setPower(MOTOR_POWER);
        leftRear.setPower(MOTOR_POWER);
        rightFront.setPower(-MOTOR_POWER);
        rightRear.setPower(-MOTOR_POWER);
    }
    void stop_bot() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    boolean search() {
        claw.setPosition(clawOpenPosition);
        moveForward();
        sleep(200);
        if(checkIfCanSee()) return true;
        moveBackward();
        sleep(500);
        if(checkIfCanSee()) return true;
        turnLeft();
        sleep(500);
        if(checkIfCanSee()) return true;
        turnRight();
        sleep(1000);
        if(checkIfCanSee()) return true;
        turnLeft();
        sleep(500);
        if(checkIfCanSee()) return true;
        stop_bot();
        return false;
    }
    boolean checkIfCaptured() {
        claw.setPosition(clawClosePosition);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);

        sleep(2000);
        // in object_classifaction its a single block, it will be either 1 return false, 2 return true
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length == 0) {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
            return false;
        }
        int numbers[] = new int[50];
        for (int i = 0; i < 50; i++) {
            numbers[i] = blocks[0].id;
        }
        //Find the number that appears the most
        int max = numbers[0];
        int maxCount = 0;
        for (int i = 0; i < 50; i++) {
            int count = 0;
            for (int j = 0; j < 50; j++) {
                if (numbers[i] == numbers[j]) {
                    count++;
                }
            }
            if (count > maxCount) {
                max = numbers[i];
                maxCount = count;
            }
        }



        if(max == 1) {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
            return false;
        } else if (max == 2){
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
            return true;
        } else {
            return false;
        }


    }

    void capturedTheBall() {
        // Make the robot spin to the left
        leftFront.setPower(-0.75);
        leftRear.setPower(-0.75);
        rightFront.setPower(0.75);
        rightRear.setPower(0.75);
        sleep(2500);
        claw.setPosition(clawOpenPosition);
        sleep(2500);
        stop_bot();





    }
}