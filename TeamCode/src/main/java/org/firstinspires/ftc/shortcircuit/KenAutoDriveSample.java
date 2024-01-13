package org.firstinspires.ftc.shortcircuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Simple autonomous program to test driving in a straight line or rotate 90 degrees.

@Autonomous(name="[Ken] Ken Auto Drive Sample", group="Ken")
//@Disabled
public class KenAutoDriveSample extends LinearOpMode
{
    KenIMUOmniDriveTrain robot = new KenIMUOmniDriveTrain(this, true);
    double MAX_DRIVE_SPEED = 0.3;
    double MAX_TURN_SPEED = 0.3;
    double targetHeading = 0; // angle in degrees

    ElapsedTime lastClick = new ElapsedTime();
    double BUTTON_DELAY_SEC = 0.25;
    ElapsedTime driveTimer = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        robot.init();

        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Ready to start.");
        telemetry.update();

        // Wait for driver to press PLAY
        waitForStart();

        // Started
        while (opModeIsActive())
        {
            // Drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            double drive  = -gamepad1.left_stick_y  * MAX_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x  * MAX_DRIVE_SPEED;
            double turn   = -gamepad1.right_stick_x * MAX_TURN_SPEED;
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            robot.moveRobot(drive, strafe, turn);

            // Rotate robot 90 degrees left/right using x/b buttons.
            // turn 90 degrees right.
            if(gamepad1.b) {
                targetHeading -= 90;
                robot.turnToHeading(MAX_TURN_SPEED, targetHeading, 1.3);
            }
            // turn 90 degrees left.
            if(gamepad1.x) {
                targetHeading += 90;
                robot.turnToHeading(MAX_TURN_SPEED, targetHeading, 1.3);
            }
            // forward
            if(gamepad1.y) {
                driveTimer.reset();
                while(opModeIsActive() && driveTimer.seconds() < 0.75) {
                    robot.cruise(MAX_DRIVE_SPEED, targetHeading);
                }
                robot.stopMotors();
            }

            // Telemetry
            telemetry.addData("Drive Speed", "%.2f", MAX_DRIVE_SPEED);
            telemetry.addData("Turn Speed", "%.2f", MAX_TURN_SPEED);
            telemetry.addData("Heading error", "%.1f", targetHeading-robot.getHeading());
            telemetry.update();
        }

        // Turn the motors off.
        robot.stopMotors();
    }
}
