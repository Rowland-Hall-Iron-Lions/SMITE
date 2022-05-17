package org.firstinspires.ftc.teamcode;

public class TestingCirclePowerMotors {
    public static void evaluate(PhysicalRobotState state) {
        state.getMotorRight().setPower(1);
        state.getMotorLeft().setPower(state.getCircleRotationDenominator());
    }

    public static void cleanup(PhysicalRobotState state) {
        state.getMotorRight().setPower(0);
        state.getMotorLeft().setPower(0);
    }
}
