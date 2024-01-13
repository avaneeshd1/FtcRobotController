/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.shortcircuit;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Claw Left: 0.46 Claw Right: 0.47

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "--> New Robot main <--", group = "Linear OpMode")
@Disabled
public class MainDriverOpmode extends LinearOpMode {


    double RobotWrist_position;
    double robot_arm_position;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor basemotor1 = null;
    private DcMotor basemotor2 = null;
    private DcMotor armmotor = null;
    private DcMotor clawmotor = null;

    private DcMotor arm_drive = null;
    private TouchSensor baselimit = null;
    private TouchSensor s1 = null;
    private TouchSensor armlimit = null;
    private TouchSensor clawlimit = null;

    private Servo clawLeft = null;
    private Servo clawRight = null;
    private Servo wrist_servo = null;
    private boolean isTank = false;
    private boolean clawClosed = false;
    private boolean isStored = false;
    static final double WRIST_INCREMENT = 0.005;
    static final int CYCLE_MS = 15;     // period of each cycle

    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double MAX_POS = 1.0;     // Maximum rotational position

    private int baseMotorZeroPosition = 0;
    private float baseMotorSpan = 60.0F;
    private int baseMotorPositions[] = new int[]{0, 20, 40, 60};
    private int basePositionsIndex = 0;

    private int armMotorZeroPosition = 0;
    private float armMotorSpan = 170.0F;

    private int clawMotorZeroPosition = 0;
    private float clawMotorSpan = 100.0F;

    private DistanceSensor sensorDistanceL;
    private DistanceSensor sensorDistanceR;

    void move() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Don't drive full power
        double speedFactor = 0.3;
        leftFrontPower  *= speedFactor;
        rightFrontPower *= speedFactor;
        leftBackPower   *= speedFactor;
        rightBackPower  *= speedFactor;  

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "drivemotor3");
        leftBackDrive = hardwareMap.get(DcMotor.class, "drivemotor4");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drivemotor1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "drivemotor2");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        basemotor1 = hardwareMap.get(DcMotorEx.class, "basemotor1");
        basemotor2 = hardwareMap.get(DcMotorEx.class, "basemotor2");
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        clawmotor = hardwareMap.get(DcMotorEx.class, "clawmotor");
        basemotor1.setDirection(DcMotor.Direction.FORWARD);
        basemotor2.setDirection(DcMotor.Direction.FORWARD);
        basemotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        basemotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armmotor.setDirection(DcMotor.Direction.FORWARD);
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawmotor.setDirection(DcMotor.Direction.REVERSE);
        clawmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        basemotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basemotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        basemotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        basemotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baselimit = hardwareMap.get(TouchSensor.class, "S0");
        s1 = hardwareMap.get(TouchSensor.class, "S1");
        armlimit = hardwareMap.get(TouchSensor.class, "S2");
        clawlimit = hardwareMap.get(TouchSensor.class, "S3");

        sensorDistanceL = hardwareMap.get(DistanceSensor.class, "TOFl");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "TOFr");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "starting");
        //calibrateBaseMotor();
        int baseMotorPosition = baseMotorZeroPosition;
        int armMotorPosition = armMotorZeroPosition;
        int clawMotorPosition = clawMotorZeroPosition;
        telemetry.addData("Reset position", "Claw:%d, Arm:%d, Base%d",
                clawMotorZeroPosition, armMotorZeroPosition, baseMotorPosition);
        telemetry.update();
        int targetPosition = 0;
        boolean rightBumperPressed = false;
        boolean leftBumperPressed = false;
        int runNumber = 0;
        while (opModeIsActive()) {
             moveArmMotorToPosition(100);
            if((runNumber % 32) == 0) {
                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString() + "Run#: " + runNumber);
                telemetry.addData("Limit", "Claw::%b, Arm::%b, Base::%b",
                        clawlimit.isPressed(),
                        armlimit.isPressed(),
                        baselimit.isPressed());
                telemetry.addData("Motor busy", "Base::%b, Arm::%b, Claw::%b",
                        basemotor1.isBusy(),
                        armmotor.isBusy(),
                        clawmotor.isBusy());
                telemetry.addData("Position", "Base::%d, Arm::%d, Claw::%d",
                        basemotor1.getCurrentPosition(),
                        armmotor.getCurrentPosition(),
                        clawmotor.getCurrentPosition());
                telemetry.addData("Target position", "Base::%d, Arm::%d, Claw::%d",
                        targetPosition,
                        armMotorPosition,
                        clawMotorPosition);
                telemetry.addData("Stick", "Lx::%f, Ly::%f, Rx::%f,Ry::%f",
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.right_stick_y);

                telemetry.update();
            }
            //sleep(CYCLE_MS);
            runNumber++;
        }
    }

    private int moveBaseMotorToPosition(int position) {
        int targetPosition = position;
        int range0 = 1;
        int range1 = 2;
        int range2 = 4;
        int range3 = 8;
        int range4 = 16;
        int motorPosition = basemotor1.getCurrentPosition();
        int delta = Math.abs(motorPosition - targetPosition);
        if(delta == 0)
            return 0;
        basemotor1.setTargetPosition(targetPosition);
        basemotor2.setTargetPosition(targetPosition);
        basemotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        basemotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = 0.0;
        if (delta == range0) {
            power = 0.01;
        } else if (delta <= range1) {
            power = 0.02;
        } else if (delta <= range2) {
            power = 0.04;
        }else if (delta <= range3) {
            power = 0.08;
        } else if (delta <= range4) {
            power = 0.16;
        }
        else {
            power = 0.25;
        }
        basemotor1.setPower(power);
        basemotor2.setPower(power);
        return delta;
    }
    private int moveArmMotorToPosition(int position) {
        int targetPosition = position;
        int range0 = 1;
        int range1 = 2;
        int range2 = 4;
        int range3 = 8;
        int range4 = 16;
        int motorPosition = armmotor.getCurrentPosition();
        int delta = Math.abs(motorPosition - targetPosition);
        if(delta == 0)
            return 0;
        armmotor.setTargetPosition(targetPosition);
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double power = 0.0;
        if (delta == range0) {
            power = 0.1;
        } else if (delta <= range1) {
            power = 0.2;
        } else if (delta <= range2) {
            power = 0.3;
        }else if (delta <= range3) {
            power = 0.4;
        } else if (delta <= range4) {
            power = 0.5;
        }
        else {
            power = 0.8;
        }
        armmotor.setPower(power);
        return delta;
    }

    void calibrateBaseMotor() {
        telemetry.addData("Calibrating", "%s, %d", "Base motor", basemotor1.getCurrentPosition());
        if (baselimit.isPressed()) {
            int motorIncrement = 1;
            while (baselimit.isPressed()) {
                basemotor1.setTargetPosition(motorIncrement);
                basemotor2.setTargetPosition(motorIncrement);
                basemotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor1.setPower(1.0);
                basemotor2.setPower(1.0);
                motorIncrement += 1;
                sleep(100);
            }
        } else {
            int motorIncrement = -1;
            while (!baselimit.isPressed()) {
                basemotor1.setTargetPosition(motorIncrement);
                basemotor2.setTargetPosition(motorIncrement);
                basemotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor1.setPower(1.0);
                basemotor2.setPower(1.0);
                motorIncrement -= 1;
                sleep(100);

            }
            motorIncrement = 1;
            while (baselimit.isPressed()) {
                basemotor1.setTargetPosition(motorIncrement);
                basemotor2.setTargetPosition(motorIncrement);
                basemotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                basemotor1.setPower(1.0);
                basemotor2.setPower(1.0);
                motorIncrement += 1;
                sleep(100);
            }
        }
        baseMotorZeroPosition = basemotor1.getCurrentPosition();
    }

    void calibrateArmMotor() {
        telemetry.addData("Calibrating", "%s, %d", "Arm motor", basemotor1.getCurrentPosition());
        if (armlimit.isPressed()) {
            int motorIncrement = 1;
            while (armlimit.isPressed()) {
                armmotor.setTargetPosition(motorIncrement);
                armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armmotor.setPower(0.2);
                motorIncrement += 1;
                sleep(CYCLE_MS);
            }
        } else {
            int motorIncrement = -1;
            while (!armlimit.isPressed()) {
                armmotor.setTargetPosition(motorIncrement);
                armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armmotor.setPower(0.2);
                motorIncrement -= 1;
                sleep(CYCLE_MS);

            }
            motorIncrement = 1;
            while (armlimit.isPressed()) {
                armmotor.setTargetPosition(motorIncrement);
                armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armmotor.setPower(0.2);
                motorIncrement += 1;
                sleep(CYCLE_MS);
            }
        }
        armMotorZeroPosition = armmotor.getCurrentPosition();
    }

    void calibrateClawMotor() {
        telemetry.addData("Calibrating", "%s, %d", "Claw motor", basemotor1.getCurrentPosition());
        if (clawlimit.isPressed()) {
            int motorIncrement = 1;
            while (armlimit.isPressed()) {
                clawmotor.setTargetPosition(motorIncrement);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawmotor.setPower(0.2);
                motorIncrement += 1;
                sleep(CYCLE_MS);
            }
        } else {
            int motorIncrement = -1;
            while (!clawlimit.isPressed()) {
                clawmotor.setTargetPosition(motorIncrement);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawmotor.setPower(0.2);
                motorIncrement -= 1;
                sleep(CYCLE_MS);

            }
            motorIncrement = 1;
            while (clawlimit.isPressed()) {
                clawmotor.setTargetPosition(motorIncrement);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                clawmotor.setPower(0.2);
                motorIncrement += 1;
                sleep(CYCLE_MS);
            }
        }
        clawMotorZeroPosition = clawmotor.getCurrentPosition();
    }
}
