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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="[Ken] 1 - Arm Joints Motor Control - Manual (Theo)", group="KenTest")
//@Disabled
public class ArmJointsMotorControl_Encoder extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor baseMotor = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;
    private TouchSensor armSensor = null;
    private TouchSensor wristSensor = null;

    boolean isArmCalibrated = false;
    boolean isWristCalibrated = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseMotor = hardwareMap.get(DcMotor.class, "basemotor");
        armMotor = hardwareMap.get(DcMotor.class, "armmotor");
        wristMotor = hardwareMap.get(DcMotor.class, "wristmotor");

        // +ve raise up; -ve lower towards ground
        baseMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armSensor = hardwareMap.get(TouchSensor.class, "armsensor");
        wristSensor = hardwareMap.get(TouchSensor.class, "wristsensor");

        // BASE MOTOR
        final int    BASE_POS_MAX_DANGER  =   +300;
        final int    BASE_POS_MAX         =   +270;
        final int    BASE_POS_DROP        =   +250;
        final int    BASE_POS_SENSOR      =      0;
        final int    BASE_POS_GROUND      =    -25;
        final int    BASE_POS_MIN         =    -50;
        final int    BASE_POS_MIN_DANGER  =   -100;

        // ARM MOTOR
        final int    ARM_POS_MAX_DANGER   = +14500; // MAX PHYSICAL LIMIT
        final int    ARM_POS_MAX          = +12000; // MAX USER
        final int    ARM_POS_DROP         = +12000;
        final int    ARM_POS_SENSOR       =      0; // REFERENCE SENSOR POSITION
        final int    ARM_POS_GROUND       =  -1000;
        final int    ARM_POS_MIN          =  -1500; // MIN USER
        final int    ARM_POS_MIN_DANGER   =  -4000; // MIN PHYSICAL LIMIT
        // ARM MOTOR CALIBRATION
        final double ARM_SEARCH_POWER     =    0.5;
        final int    ARM_SEARCH_1_DELTA   = ARM_POS_SENSOR - ARM_POS_MIN; // SEARCH UP    +1500
        final int    ARM_SEARCH_2_DELTA   = ARM_POS_SENSOR - ARM_POS_MAX; // SEARCH DOWN -12000
        final double ARM_SEARCH_1_TIMEOUT =    1.0/ARM_SEARCH_POWER; // FOR FULL POWER
        final double ARM_SEARCH_2_TIMEOUT =   10.0/ARM_SEARCH_POWER;
        final double ARM_SENSOR_SPAN      =    700;
        final int    ARM_POS_WRIST_CAL    = ARM_POS_SENSOR - ARM_POS_GROUND; // ARM POSITION FOR WRIST TO ACCESS RANGE BELOW GROUND +1000

        // WRIST MOTOR
        final int    WRIST_POS_MAX_DANGER =  +3500; // MAX PHYSICAL LIMIT
        final int    WRIST_POS_MAX        =  +3300; // MAX USER
        final int    WRIST_POS_DROP       =  +3200;
        final int    WRIST_POS_GROUND     =   +550; // ARM NEEDS TO BE ELEVATED FOR ACCESSING RANGE BELOW GROUND
        final int    WRIST_POS_MIN        =   +500; // MIN USER
        final int    WRIST_POS_SENSOR     =      0; // REFERENCE SENSOR POSITION --> BELOW GROUND / MIN USER
        final int    WRIST_POS_MIN_DANGER =   -800;
        // WRIST MOTOR CALIBRATION
        final int    WRIST_SENSOR_SPAN    =    200;
        final double WRIST_SEARCH_POWER   =    0.5;
        final int    WRIST_SEARCH_1_DELTA =   -WRIST_SENSOR_SPAN+ WRIST_POS_SENSOR - WRIST_POS_MAX_DANGER; // SEARCH DOWN -3500
        final double WRIST_SEARCH_1_TIMEOUT =  100;   // 1000/sec at 0.5 PWR
        // WRIST REFERENCE POSITION IS ALWAYS IN THE NEGATIVE

        // WRIST CALIBRATION STEPS:
        // 1. RAISE ARM TO CLEAR THE GROUND: ARM_POS_WRIST_CAL
        // 2. SEARCH NEGATIVE: WRIST_POS_SENSOR-WRIST_POS_MAX_DANGER

        telemetry.addData("Sensor state", "Arm: %b  Wrist: %b", armSensor.isPressed(), wristSensor.isPressed());
        telemetry.addData("Current base position", baseMotor.getCurrentPosition());
        telemetry.addData("Current arm position", armMotor.getCurrentPosition());
        telemetry.addData("Current wrist position", wristMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        baseMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        baseMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double basePower  = (-gamepad1.left_trigger + gamepad1.right_trigger) * 0.5; // TODO: CALIBRATE MAX POWER FOR BASE
            double armPower   = -gamepad1.left_stick_y  * 0.5; // Note: pushing stick forward gives negative stick_y value
            double wristPower = -gamepad1.right_stick_y * 0.5; // Note: pushing stick forward gives negative stick_y value

            boolean armSensorState = armSensor.isPressed();
            boolean wristSensorState = wristSensor.isPressed();

            baseMotor.setPower(basePower);
            armMotor.setPower(armPower);
            wristMotor.setPower(wristPower);

            // RESET ENCODERS WHOSE SENSOR IS TRUE
            if (gamepad1.y) {
                if (armSensor.isPressed())
                {
                    isArmCalibrated = true;
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (wristSensor.isPressed())
                {
                    isWristCalibrated = true;
                    wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            // Search for reference for arm motor
            if (gamepad1.a) {
                // Arm Calibration
                // 1. Make sure the wrist is not too low

                if (!armSensor.isPressed()) {
                    telemetry.addData(">", "Calibrate arm - Searching UP...");
                    telemetry.update();
                    boolean found = SearchReferencePosition(armMotor, armSensor, ARM_SEARCH_1_DELTA, ARM_SEARCH_POWER, ARM_SEARCH_1_TIMEOUT);
//                    if (found) {
//                        telemetry.addLine(">Press A to continue...");
//                        telemetry.update();
//                        while (!gamepad1.a) sleep(100);
//                    }
                    if (!found) {
                        telemetry.addData(">", "Calibrate arm - Searching Down...");
                        telemetry.update();
                        found = SearchReferencePosition(armMotor, armSensor, -ARM_SEARCH_1_DELTA + ARM_SEARCH_2_DELTA, ARM_SEARCH_POWER, ARM_SEARCH_2_TIMEOUT);
//                        telemetry.addLine(">Press A to continue...");
//                        telemetry.update();
////                        while (!gamepad1.a) sleep(100);
                    }
                }

//                ElapsedTime searchTimer = new ElapsedTime();
//                searchTimer.reset();
//                armMotor.setPower(0.75);
//                while (true) {
//                    if (armSensor.isPressed() || gamepad1.b || searchTimer.seconds() >= 10) {
//                        armMotor.setPower(0);
//                        break;
//                    }
//                    telemetry.addData(">", "Searching the arm reference position");
//                    telemetry.addData("Search Time", searchTimer.seconds());
//                    if (searchTimer.seconds() >= 1.5) {
//                        armMotor.setPower(-0.75);
//                    }
//                    telemetry.update();
//                }
            }
            // Search for reference for wrist motor
            if (gamepad1.x) {
                if (!wristSensor.isPressed()) {
                    telemetry.addData(">", "Calibrate wrist - Searching DOWN...");
                    telemetry.update();
                    boolean found = SearchReferencePosition(wristMotor, wristSensor, WRIST_SEARCH_1_DELTA, WRIST_SEARCH_POWER, 5);
//                    if (found) {
//                        telemetry.addLine(">Press X to continue...");
//                        telemetry.update();
//                        while (!gamepad1.x) sleep(100);
//                        sleep(500);
//                    }
                }
            }


            telemetry.addData("BASE ", "Pos=%d  Pwr=%.1f",             baseMotor.getCurrentPosition(),  baseMotor.getPower());
            telemetry.addData("ARM  ", "Pos=%d  Pwr=%.1f\tSense=%.1f", armMotor.getCurrentPosition(),   armMotor.getPower(),   armSensor.getValue());
            telemetry.addData("WRIST", "Pos=%d  Pwr=%.1f\tSense=%.1f", wristMotor.getCurrentPosition(), wristMotor.getPower(), wristSensor.getValue());
            telemetry.addData("Found hole (Arm)",   armSensorState);
            telemetry.addData("Found hole (Wrist)", wristSensorState);
            telemetry.addData("Arm Calibrated",     isArmCalibrated);
            telemetry.addData("Wrist Calibrated",   isWristCalibrated);
            telemetry.update();
        }
    }

    private boolean SearchReferencePosition(
            DcMotor motor,
            TouchSensor sensor,
            int searchDelta,        // search from current pos
            double maxPower,        // 0..1
            double timeout)         // seconds
    {
        if (sensor.isPressed()) // Already at reference position
            return true;

        // Determine new target position, and pass to motor controller
        int startPos = motor.getCurrentPosition();
        int targetPos = startPos + searchDelta;
//        int lastPos = startPos;

        int[] edgePos = new int[]{0, 0};
        int edgeIndex = 0;
        boolean edgeTarget = true; // First sensor value to look for

        ElapsedTime searchTime = new ElapsedTime();
        searchTime.reset();

        // Set Target FIRST, then turn on RUN_TO_POSITION
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required motor speed (must be positive for RUN_TO_POSITION)
        motor.setPower(Math.abs(maxPower));

//        boolean started = false;

        // Keep looping while we are still active, and motor is running.
        while(opModeIsActive() && motor.isBusy()) {
            int curPos = motor.getCurrentPosition();
            boolean curState = sensor.isPressed();
            double speed = (curPos - startPos) / searchTime.seconds();

            // Sensor target
            if (edgeIndex < 2 && curState == edgeTarget) { // Found an edge
//                telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
//                        searchTime.seconds(), curPos, curState,
//                        speed, motor.getPower());

                edgePos[edgeIndex] = curPos;
                telemetry.addData(">", "Goal Reached @%d = %b", edgePos[edgeIndex], edgeTarget);

                // Invert target for the next edge
                edgeTarget = !edgeTarget;
                ++edgeIndex;

                // Found both edges
                if (edgeIndex == 2) {
                    targetPos = (edgePos[0] + edgePos[1])/2;
                    motor.setTargetPosition(targetPos);
                    telemetry.addData(">", "HOLE SIDES @ [%d , %d] Width =  %d",
                            edgePos[0], edgePos[1], Math.abs(edgePos[1]-edgePos[0]));
                }
            }
//            // Jammed
//            started |= curPos != lastPos;
//            if (started && curPos == lastPos) {
//                telemetry.addLine("Jammed");
//                break;
//            }
            // Emergency Stop
            if (gamepad1.b) {
                telemetry.addLine("Emergency Stop");
                speed = (motor.getCurrentPosition() - startPos) / searchTime.seconds();
                telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
                        searchTime.seconds(), motor.getCurrentPosition(), sensor.isPressed(),
                        speed, motor.getPower());
                telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
                telemetry.update(); while (!gamepad1.dpad_right) sleep(100);
                break;
            }
            // Timeout
            if (searchTime.seconds() > timeout) {
                telemetry.addLine("Timeout");
                break;
            }

            // Update last position to current
//            lastPos = curPos;
        }

        telemetry.addData(">", "%.1f sec, LOOP EXIT(opActive=%b motorBusy=%b)",
                searchTime.seconds(), opModeIsActive(), motor.isBusy());
        telemetry.addData("Power", "%.2f", motor.getPower());

        motor.setPower(0);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);

        boolean success = sensor.isPressed();
        telemetry.addData("-------->", success ? "SUCCESS" : "SEARCH FAILED");
        telemetry.addData(">DONE CALIB", "STARTED @%d  STOPPED @%d", startPos, motor.getCurrentPosition());
//        telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
//        telemetry.update(); while (!gamepad1.dpad_right) sleep(100);

        return success;
    }
}
