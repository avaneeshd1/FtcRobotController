package org.firstinspires.ftc.shortcircuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="[Ken] Arm Joints Test", group="Ken")
//@Disabled
public class KenArmJointsTestOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    KenArmJoints armJoints = new KenArmJoints(this);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize components.
        armJoints.init();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Start components.
        armJoints.start();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double basePower  = (-gamepad1.left_trigger + gamepad1.right_trigger) * 0.10;
            double armPower   = -gamepad1.left_stick_y  * 0.75; // Note: pushing stick forward gives negative stick_y value
            double wristPower = -gamepad1.right_stick_y * 0.75; // Note: pushing stick forward gives negative stick_y value
            armJoints.actuate(basePower, armPower, wristPower);

             if (gamepad1.dpad_left) {
                armJoints.goToPose(KenArmJoints.Pose.GROUND);
            } else if (gamepad1.dpad_right) {
                armJoints.goToPose(KenArmJoints.Pose.DROP);
            } else if (gamepad1.back) {
                 armJoints.goToPose(KenArmJoints.Pose.PARKED);
             }

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Target Power", "Base::%.1f, Arm::%.1f, Wrist::%.1f",
                    basePower, armPower, wristPower);
            armJoints.addTelemetry();
            telemetry.addData(">", "Parked Pose: BACK");
            telemetry.addData(">", "Ground Pose: DPAD_LEFT");
            telemetry.addData(">", "BackDrop Pose: DPAD_RIGHT");
            telemetry.update();
        }
    }
}
