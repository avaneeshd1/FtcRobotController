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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@TeleOp(name="[Ken] 2 - Arm Joints Motor Control - PID (Theo)", group="Linear OpMode")
@Disabled
public class ArmJointsMotorControl_PID extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armDrive = null;
    private DcMotor wristDrive = null;
    private double armMotorPower = 0.5;
    private double wristMotorPower = 0.7;
    private int armMotorTarget = 0;
    private int wristMotorTarget = 0;
    private boolean foundHoleArm = false;
    private boolean foundHoleWrist = false;
    TouchSensor armSensor;
    TouchSensor wristSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        armDrive = hardwareMap.get(DcMotor.class, "armmotor");
        wristDrive = hardwareMap.get(DcMotor.class, "wristmotor");

        armSensor = hardwareMap.get(TouchSensor.class, "armsensor");
        wristSensor = hardwareMap.get(TouchSensor.class, "wristsensor");

//        double ARM_MAX = 17500;
//        double ARM_MIN = 0;
//
//        double WRIST_MIN = 0;
//        double WRIST_MAX = 730;

        telemetry.addData("Current arm position", "%10d", armDrive.getCurrentPosition());
        telemetry.addData("Current wrist position", "%10d", wristDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        wristDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        armDrive.setTargetPosition(armMotorTarget);
        wristDrive.setTargetPosition(wristMotorTarget);

        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            armDrive.setTargetPosition(armMotorTarget);
            wristDrive.setTargetPosition(wristMotorTarget);

            armMotorTarget += gamepad1.left_stick_y * -10;
            wristMotorTarget += gamepad1.right_stick_y * 7;

            wristDrive.setPower(wristMotorPower);
            armDrive.setPower(armMotorPower);

            armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            foundHoleArm = armSensor.isPressed();
            foundHoleWrist = wristSensor.isPressed();

            telemetry.addData("Current arm motor power", "%5f", armDrive.getPower());
            telemetry.addData("Current arm position", "%10d", armDrive.getCurrentPosition());
            telemetry.addData("Current wrist motor power", "%5f", wristDrive.getPower());
            telemetry.addData("Current wrist position", "%10d", wristDrive.getCurrentPosition());
            telemetry.addData("Found hole (Arm)", foundHoleArm);
            telemetry.addData("Found hole (Wrist)", foundHoleWrist);
            telemetry.update();
        }
    }
}
