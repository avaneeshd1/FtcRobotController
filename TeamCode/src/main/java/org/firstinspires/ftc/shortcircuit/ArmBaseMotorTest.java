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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="[Ken] Arm Base Motor Test (Theo)", group="Linear OpMode")
@Disabled
public class ArmBaseMotorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lastBumperButtonPress = new ElapsedTime();
    private ElapsedTime lastTriggerButtonPress = new ElapsedTime();
    private DcMotor baseMotor = null;

    private final int MIN_ARM_POS = 0;
    private final int MAX_ARM_POS = 1000000;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        baseMotor = hardwareMap.get(DcMotor.class, "basemotor");

        //Run the base motor using the encoder
        baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        baseMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        baseMotor.setTargetPosition(0);
        baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Approximate target for encoder, controlled by joystick
        float userBaseEncoderTarget = 0;
        //Exact target, can change to prevent oscillation but must always be within the encoder tolerance of the vague target
        float applyBaseEncoderTarget = 0;
        //Change the tolerance for how far the arm can move beyond the set target position
        int encoderTolerance = 0;
        //The power for the motor
        float basePower = 0.5F;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // userBaseEncoderTarget
            userBaseEncoderTarget += gamepad1.left_stick_y * -1;
            if(gamepad1.left_stick_button){
                userBaseEncoderTarget = 0;
            } else if (gamepad1.right_stick_button) {
                userBaseEncoderTarget = 400;
            }

            userBaseEncoderTarget = clamp(userBaseEncoderTarget, (float)MIN_ARM_POS, (float)MAX_ARM_POS);

            // encoderTolerance
            if (lastBumperButtonPress.seconds() > 0.25) {
                if (gamepad1.left_bumper) {
                    encoderTolerance += 1;
                    lastBumperButtonPress.reset();
                } else if (gamepad1.right_bumper) {
                    encoderTolerance -= 1;
                    lastBumperButtonPress.reset();
                }
                encoderTolerance = clamp(encoderTolerance, 0, 100);
            }

            //Change the power for the motor with the triggers
            if (lastTriggerButtonPress.seconds() > 0.25) {
                if (gamepad1.left_trigger > 0) {
                    basePower += 0.1;
                    lastTriggerButtonPress.reset();
                } else if (gamepad1.right_trigger > 0) {
                    basePower -= 0.1;
                    lastTriggerButtonPress.reset();
                }
                basePower = clamp(basePower, 0F, 1F);
            }

            // applyBaseEncoderTarget
            if (Math.abs(userBaseEncoderTarget - baseMotor.getCurrentPosition()) < encoderTolerance) {
                applyBaseEncoderTarget = baseMotor.getCurrentPosition();
                baseMotor.setPower(0);
            }
            else {
                applyBaseEncoderTarget = userBaseEncoderTarget;
                baseMotor.setTargetPosition((int)applyBaseEncoderTarget);
                baseMotor.setPower(basePower);
            }

            baseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("User target position", "%.2f", userBaseEncoderTarget);
            telemetry.addData("Apply target position", "%.2f", applyBaseEncoderTarget);
            telemetry.addData("Current position", "%d", baseMotor.getCurrentPosition());
            telemetry.addData("Current encoder tolerance", "%d", encoderTolerance);
            telemetry.addData("Current Power", "%.2f", baseMotor.getPower());
            telemetry.addData("Set power", "%.2f", basePower);
            telemetry.update();
        }
    }

    public static <T extends Number & Comparable<T>> T clamp(T value, T min, T max) {
        if (value.compareTo(min) < 0) {
            return min;
        } else if (value.compareTo(max) > 0) {
            return max;
        } else {
            return value;
        }
    }

}
