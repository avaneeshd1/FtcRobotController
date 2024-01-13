

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

public class KenClaw {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */

    public KenClaw(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // Define class members
    Servo right_Servo;
    Servo left_Servo;
    double left_arm_open_position = .35; // at port 0 (more = out)?
    double right_arm_open_position = .62; // at port 1
    double left_arm_closed_position = .2; // left test (moves in when smaller)
    double right_arm_closed_position = .70;


    public void init() {
        right_Servo = opMode.hardwareMap.get(Servo.class, "rightClaw");
        left_Servo = opMode.hardwareMap.get(Servo.class, "leftClaw");

        left_Servo.setPosition(left_arm_open_position);
        right_Servo.setPosition(right_arm_open_position);

        // Wait for the start button
        opMode.telemetry.addData(">", "Press Start to scan Servo.");
        opMode.telemetry.update();
    }


    public void open() {
        left_Servo.setPosition(left_arm_open_position);
        right_Servo.setPosition(right_arm_open_position);
    }

    public void closed() {
        left_Servo.setPosition(left_arm_closed_position);
        right_Servo.setPosition(right_arm_closed_position);
    }

    public void toggle_left() {
        if (left_Servo.getPosition() == .35) {
            left_Servo.setPosition(left_arm_closed_position);
        } else {
            left_Servo.setPosition(left_arm_open_position);
        }
//        sleep(200);   caller calls the sleep
    }

    public void toggle_right() {
        if (right_Servo.getPosition() == .62) {
            right_Servo.setPosition(right_arm_closed_position);
        } else {
            right_Servo.setPosition(right_arm_open_position);
        }

//        sleep(200);
    }
}
