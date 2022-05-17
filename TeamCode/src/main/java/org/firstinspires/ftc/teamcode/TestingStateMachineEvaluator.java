package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Basic: First State Machine Evaluation TeleOp.", group="Regular Opmode")
public class TestingStateMachineEvaluator extends OpMode {
    private PhysicalRobotState robotState = null;
    private TestingState state = TestingState.GLOBAL_ENTRY;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robotState = new PhysicalRobotState(
            hardwareMap.get(DcMotor.class, "motorLeft"),
            hardwareMap.get(DcMotor.class, "motorRight"),
            hardwareMap.get(DistanceSensor.class, "distanceLeft"),
            hardwareMap.get(DistanceSensor.class, "distanceRight"),
            hardwareMap.get(Servo.class, "servo")
        );

        robotState.setDefaults();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robotState.getElapsedTime().reset();
    }

    @Override
    public void loop() {
        telemetry.addLine(this.state.name());

        // Loop until we reach the end state.
        switch (this.state) {
            case GLOBAL_ENTRY:
                // TODO: Add logic to run specific sub-FSM at runtime instead of
                //  hardcoding the value.

                this.state = TestingState.CIRCLE_ENTRY;
                break;
            case CIRCLE_ENTRY:
                // Set circle power ratio.
                robotState.setCircleRotationDenominator(0.1f);

                this.state = TestingState.POWERING_MOTORS;

                break;
            case POWERING_MOTORS:
                // TODO: We are ignoring the conditional here just because we want to get the robot
                // TODO: moving. In future, this should detect if we have already traveled a full
                // TODO: circle, or more (hopefully not though).
                if (false) {
                    TestingCirclePowerMotors.cleanup(robotState);
                    this.state = TestingState.GLOBAL_END;
                }

                TestingCirclePowerMotors.evaluate(robotState);

                break;
            case MAZE_ENTRY:
                break;
            case SETTING_SERVO_DIRECTION:
                break;
            case CLASSIC_COBB_SALAD_DRIVING:
                break;
            case CLASSIC_COBB_SALAD_TURNING:
                break;
            case S_REGULATING_ANGLE:
                break;
            case GLOBAL_END:
                break;
            default:
                telemetry.addData("Something bad", "Unknown state.");

                break;
        }

        // Break out of the state loop if we are in the end state.
        if (this.state == TestingState.GLOBAL_END) {
            telemetry.addData("Status", "DIE YOU HEATHEN");
        }

        telemetry.addLine(this.state.toString());

        /* try {
            Thread.sleep(500);
        } catch (InterruptedException ignored) {
            telemetry.addLine("Sleep failed.");
        } */

        telemetry.addLine(robotState.toString());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    
    }
}
