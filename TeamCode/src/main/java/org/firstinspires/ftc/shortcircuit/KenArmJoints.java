package org.firstinspires.ftc.shortcircuit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class KenArmJoints {
    private LinearOpMode opMode;   // gain access to methods in the calling OpMode.

    /* Declare Component members. */

    // Motors
    private DcMotor baseMotor = null;
    private DcMotor armMotor = null;
    private DcMotor wristMotor = null;

    // Reference Point Sensors
    private TouchSensor armSensor = null;
    private TouchSensor wristSensor = null;

    boolean isArmCalibrated = false;
    boolean isWristCalibrated = false;


    public KenArmJoints(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    // Initialize all hardware components.
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseMotor  = opMode.hardwareMap.get(DcMotor.class, "basemotor");
        armMotor   = opMode.hardwareMap.get(DcMotor.class, "armmotor");
        wristMotor = opMode.hardwareMap.get(DcMotor.class, "wristmotor");

        // +ve raise up; -ve lower towards ground
        baseMotor .setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor  .setDirection(DcMotorSimple.Direction.FORWARD);
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armSensor   = opMode.hardwareMap.get(TouchSensor.class, "armsensor");
        wristSensor = opMode.hardwareMap.get(TouchSensor.class, "wristsensor");

        // We do not automatically reset the encoders in case they are already calibrated
        // relative to the reference position given by the optical sensors.
        baseMotor .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor  .setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motors to BRAKE mode
        baseMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Start all hardware components.
    public void start(){
        if(!tryResetEncoders()) {
//            opMode.telemetry.addLine("Start calibration?  A::[OK]   B::[SKIP]");
//            opMode.telemetry.update();
//            while(opMode.opModeIsActive() && !opMode.gamepad1.b){
//                if(opMode.gamepad1.a) {
                    calibrateEncoders();
//                    break;
//                }
//            }
        }
    }

    public void addTelemetry(){
        opMode.telemetry.addData("Current position", "Base::%d, Arm::%d, Wrist::%d",
                baseMotor.getCurrentPosition(), armMotor.getCurrentPosition(), wristMotor.getCurrentPosition());
        opMode.telemetry.addData("Motor power", "Base::%.1f, Arm::%.1f, Wrist::%.1f",
                baseMotor.getPower(), armMotor.getPower(), wristMotor.getPower());
        opMode.telemetry.addData("Motor busy", "Base::%b, Arm::%b, Wrist::%b",
                baseMotor.isBusy(), armMotor.isBusy(), wristMotor.isBusy());
        opMode.telemetry.addData("Motor sensor", "Arm::%b, Wrist::%b",
                armSensor.isPressed(), wristSensor.isPressed());
        opMode.telemetry.addData("Calibrated", "Arm::%b, Wrist::%b",
                isArmCalibrated, isWristCalibrated);
    }

    public void actuate(double basePower, double armPower, double wristPower) {
        int basePos  = baseMotor.getCurrentPosition();
        int armPos   = armMotor.getCurrentPosition();
        int wristPos = wristMotor.getCurrentPosition();

        // If moving past safe limits, cut the power
        if (isArmCalibrated) {
            armPower = cutPowerIfMovingPastLimits(armPower, armPos, ARM_POS_MIN, ARM_POS_MAX);
            basePower = cutPowerIfMovingPastLimits(basePower, basePos, BASE_POS_MIN, BASE_POS_MAX);
        }
        if (isWristCalibrated) {
            wristPower = cutPowerIfMovingPastLimits(wristPower, wristPos, WRIST_POS_MIN, WRIST_POS_MAX);
        }

        double maxBasePower  = BASE_POWER;
        double maxArmPower   = isArmCalibrated   ? ARM_RUN_POWER   : ARM_SEARCH_POWER;
        double maxWristPower = isWristCalibrated ? WRIST_RUN_POWER : WRIST_SEARCH_POWER;

        baseMotor .setPower(Range.clip(basePower , -maxBasePower,  maxBasePower));
        armMotor  .setPower(Range.clip(armPower  , -maxArmPower,   maxArmPower));
        wristMotor.setPower(Range.clip(wristPower, -maxWristPower, maxWristPower));
    }

    private double cutPowerIfMovingPastLimits(double power, int pos, int minPos, int maxPos) {
        if ((pos < minPos && power < 0) || (pos > maxPos && power > 0))
            return 0;
        return power;
    }

    public enum Pose {
        PARKED,
        GROUND,
        DROP,
    }

    public void goToPose(Pose pose)
    {
        switch (pose) {
            case PARKED:
                goToPose(pose, BASE_POS_MIN, ARM_POS_SENSOR, WRIST_POS_PARKED);
                break;
            case GROUND:
                goToPose(pose, BASE_POS_MIN, ARM_POS_GROUND, WRIST_POS_GROUND);
                break;
            case DROP:
                goToPose(pose, BASE_POS_MAX, ARM_POS_DROP, WRIST_POS_DROP);
                break;
        }
    }

    private void goToPose(Pose pose, int basePos, int armPose, int wristPos) {
        if (isArmCalibrated && isWristCalibrated) {
            // Start the motors
            startMotorTargetPosition(baseMotor,  basePos,  BASE_POWER);
            startMotorTargetPosition(armMotor,   armPose,  ARM_RUN_POWER);
            startMotorTargetPosition(wristMotor, wristPos, WRIST_RUN_POWER);
            // Keep looping while we are still active, and motor is running.
            while(opMode.opModeIsActive() &&
                    (baseMotor.isBusy() || armMotor.isBusy() || wristMotor.isBusy()))
            {
                if (opMode.gamepad1.dpad_up||opMode.gamepad1.dpad_down)
                    break;

                opMode.telemetry.addData("AUTO Strike a Pose ... ", pose);
                opMode.telemetry.addData(">", "[Cancel] DPAD Up/Down");
                opMode.telemetry.addLine("---------------------------");
                addTelemetry();
                opMode.telemetry.update();
            }
            // Stop the motors
            stopMotor(baseMotor);
            stopMotor(armMotor);
            stopMotor(wristMotor);
        }
    }

    private boolean tryResetEncoders(){
        if (armSensor.isPressed())
        {
            isArmCalibrated = true;
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // TODO: WHAT ABOUT THE BASE MOTOR? DOES IT NEED STOP_AND_RESET_ENCODER?
            // When the arm is at reference position, gravity makes the base motor at min position.
            baseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            baseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (wristSensor.isPressed())
        {
            isWristCalibrated = true;
            wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return (armSensor.isPressed() && wristSensor.isPressed());
    }

    private void calibrateEncoders(){
            opMode.telemetry.addLine("Performing Arm Joint calibration ...");
            opMode.telemetry.update();

        int armStartPos = armMotor.getCurrentPosition();
        int wristStartPos = wristMotor.getCurrentPosition();

        // Arm Calibration
        if (!isArmCalibrated && !armSensor.isPressed()) {
        // 1a. If ARM not calibrated, Search UP
            if (!searchReferencePosition(armMotor, armSensor, ARM_SEARCH_1_DELTA, ARM_SEARCH_POWER, ARM_SEARCH_1_TIMEOUT)){
            // 2. If ARM still not calibrated, make sure the WRIST is not too low before Searching DOWN for ARM
                runMotorFromCurrent(wristMotor, WRIST_SAFE_UP, WRIST_SEARCH_POWER);
                searchReferencePosition(armMotor, armSensor, -ARM_SEARCH_1_DELTA + ARM_SEARCH_2_DELTA, ARM_SEARCH_POWER, ARM_SEARCH_2_TIMEOUT);
                // Revert wrist position
                runMotorFromCurrent(wristMotor, -WRIST_SAFE_UP, WRIST_SEARCH_POWER);
            }
        } else {
            // 1b. If ARM is CALIBRATED, RAISE ARM TO CLEAR THE GROUND
            runMotorToPosition(armMotor, ARM_POS_SENSOR, WRIST_SEARCH_POWER);

        }

        // Wrist Calibration
        if (!isWristCalibrated && !wristSensor.isPressed()) {
        // 1a. If WRIST not calibrated, Search DOWN
            searchReferencePosition(wristMotor, wristSensor, WRIST_SEARCH_1_DELTA, WRIST_SEARCH_POWER, WRIST_SEARCH_1_TIMEOUT);
        }

        // If successful, move back up to safe range since wrist reference position is below ground
        goToPose(Pose.PARKED);
    }

    private void runMotorFromCurrent(DcMotor motor, int delta, double maxPower) {
        int targetPos = motor.getCurrentPosition() + delta;
        runMotorToPosition(motor, targetPos, maxPower);
    }

    private void runMotorToPosition(DcMotor motor, int targetPos, double maxPower) {
        // Start the motor
        startMotorTargetPosition(motor, targetPos, maxPower);
        // Keep looping while we are still active, and motor is running.
        while(opMode.opModeIsActive() && motor.isBusy()) {
            opMode.sleep(CYCLE_MS);
        }
        // Stop the motor
        stopMotor(motor);
    }

    private void startMotorTargetPosition(DcMotor motor, int targetPos, double maxPower) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required motor speed (must be positive for RUN_TO_POSITION)
        motor.setPower(Math.abs(maxPower));
    }

    private void stopMotor(DcMotor motor) {
        // NOTE: Wrist Motor was difficult to get it to stop.
        // Steps below may not be optimal, but was the first combination found to work.
        // TODO: Understand how to properly stop a motor in RunMode.RUN_TO_POSITION
        motor.setPower(0);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    private boolean searchReferencePosition(
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

        int[] edgePos = new int[]{0, 0};
        int edgeIndex = 0;
        boolean edgeTarget = true; // First sensor value to look for

        ElapsedTime searchTime = new ElapsedTime();
        searchTime.reset();

        // Set Target FIRST, then turn on RUN_TO_POSITION, with the required motor speed
        startMotorTargetPosition(motor, targetPos, maxPower);

        // Keep looping while we are still active, and motor is running.
        while(opMode.opModeIsActive() && motor.isBusy()) {
            int curPos = motor.getCurrentPosition();
            boolean curState = sensor.isPressed();
            double speed = (curPos - startPos) / searchTime.seconds();

            // Sensor target
            if (edgeIndex < 2 && curState == edgeTarget) { // Found an edge
//                telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
//                        searchTime.seconds(), curPos, curState,
//                        speed, motor.getPower());

                edgePos[edgeIndex] = curPos;
                opMode.telemetry.addData(">", "Goal Reached @%d = %b", edgePos[edgeIndex], edgeTarget);

                // Invert target for the next edge
                edgeTarget = !edgeTarget;
                ++edgeIndex;

                // Found both edges
                if (edgeIndex == 2) {
                    targetPos = (edgePos[0] + edgePos[1])/2;
                    motor.setTargetPosition(targetPos);
                    opMode.telemetry.addData(">", "HOLE SIDES @ [%d , %d] Width =  %d",
                            edgePos[0], edgePos[1], Math.abs(edgePos[1]-edgePos[0]));
                }
            }
            // Emergency Stop
            if (opMode.gamepad1.b) {
                opMode.telemetry.addLine("Emergency Stop");
                speed = (motor.getCurrentPosition() - startPos) / searchTime.seconds();
                opMode.telemetry.addData(">", "%.1f @ %d = %b  SPD=%.1f/s @ %.2fPWR",
                        searchTime.seconds(), motor.getCurrentPosition(), sensor.isPressed(),
                        speed, motor.getPower());
                opMode.telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
                opMode.telemetry.update(); while (!opMode.gamepad1.dpad_right) opMode.sleep(100);
                break;
            }
            // Timeout
            if (searchTime.seconds() > timeout) {
                opMode.telemetry.addLine("Timeout");
                break;
            }
        }

        opMode.telemetry.addData(">", "%.1f sec, LOOP EXIT(opActive=%b motorBusy=%b)",
                searchTime.seconds(), opMode.opModeIsActive(), motor.isBusy());
        opMode.telemetry.addData("Power", "%.2f", motor.getPower());

        stopMotor(motor);

        boolean success = sensor.isPressed();
        opMode.telemetry.addData("-------->", success ? "SUCCESS" : "SEARCH FAILED");
        opMode.telemetry.addData(">DONE CALIB", "STARTED @%d  STOPPED @%d", startPos, motor.getCurrentPosition());
//        telemetry.addLine("PRESS DPAD_RIGHT TO CONTINUE");
//        telemetry.update(); while (!gamepad1.dpad_right) sleep(100);

        if (success){
            tryResetEncoders();
        }
        return success;
    }

    // BASE MOTOR
    final int    BASE_POS_MAX         =  +120;
    final int    BASE_POS_MIN         =     0;
    final double BASE_POWER           =   0.1;
    // ARM MOTOR
    final int    ARM_POS_MAX_DANGER   = +14500; // MAX PHYSICAL LIMIT
    final int    ARM_POS_MAX          = +12000; // MAX USER
    final int    ARM_POS_DROP         = +12000;
    final int    ARM_POS_SENSOR       =      0; // REFERENCE SENSOR POSITION
    final int    ARM_POS_GROUND       =  -1400;
    final int    ARM_POS_MIN          =  -1500; // MIN USER
    final int    ARM_POS_MIN_DANGER   =  -4000; // MIN PHYSICAL LIMIT
    // ARM MOTOR CALIBRATION
    final double ARM_SEARCH_POWER     =    0.5;
    final double ARM_RUN_POWER        =    0.75;
    final int    ARM_SEARCH_1_DELTA   = ARM_POS_SENSOR - ARM_POS_MIN; // SEARCH UP    +1500
    final int    ARM_SEARCH_2_DELTA   = ARM_POS_SENSOR - ARM_POS_MAX; // SEARCH DOWN -12000
    final double ARM_SEARCH_1_TIMEOUT =    2.0/ARM_SEARCH_POWER; // FOR FULL POWER
    final double ARM_SEARCH_2_TIMEOUT =   10.0/ARM_SEARCH_POWER;
    final double ARM_SENSOR_SPAN      =    700;

    // WRIST MOTOR
    final int    WRIST_POS_MAX_DANGER =  +3500; // MAX PHYSICAL LIMIT
    final int    WRIST_POS_MAX        =  +3300; // MAX USER
    final int    WRIST_POS_DROP       =  +3200;
    final int    WRIST_POS_PARKED     =  WRIST_POS_MAX;
    final int    WRIST_POS_GROUND     =   +500; // ARM NEEDS TO BE ELEVATED FOR ACCESSING RANGE BELOW GROUND
    final int    WRIST_POS_MIN        =   +450; // MIN USER
    final int    WRIST_POS_SENSOR     =      0; // REFERENCE SENSOR POSITION --> BELOW GROUND / MIN USER
    final int    WRIST_POS_MIN_DANGER =   -800;
    // WRIST MOTOR CALIBRATION
    final int    WRIST_SENSOR_SPAN    =    200;
    final double WRIST_SEARCH_POWER   =    0.5;
    final double WRIST_RUN_POWER      =    0.75;
    final int    WRIST_SEARCH_1_DELTA =   -WRIST_SENSOR_SPAN+ WRIST_POS_SENSOR - WRIST_POS_MAX_DANGER; // SEARCH DOWN -3500
    final int    WRIST_SAFE_UP        = WRIST_POS_MAX_DANGER - WRIST_POS_MAX;
    final double WRIST_SEARCH_1_TIMEOUT =  5;   // 1000/sec at 0.5 PWR
    // WRIST REFERENCE POSITION IS ALWAYS IN THE NEGATIVE

    static final int CYCLE_MS = 15;     // period of each cycle

}
