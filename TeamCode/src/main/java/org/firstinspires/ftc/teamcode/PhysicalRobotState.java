package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This could be a singleton but... eh.
public class PhysicalRobotState {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private DcMotor motorLeft = null;
    private DcMotor motorRight = null;
    private DistanceSensor distanceSensorLeft = null;
    private DistanceSensor distanceSensorRight = null;
    private Servo servoDistanceSensor = null;

    // Describes the motor power ratio for the circle task.
    // The denominator in the circle rotation ratio.
    // The numerator is the power of the right motor (1),
    // whereas the denominator is the power of the left
    // motor.
    private float circleRotationDenominator = 0.1f;

    /** Constructs the robot's physical state. Does not initialize parameters. */
    public PhysicalRobotState(DcMotor motorLeft, DcMotor motorRight, DistanceSensor distanceSensorLeft,
        DistanceSensor distanceSensorRight, Servo servoDistanceSensor) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.distanceSensorLeft = distanceSensorLeft;
        this.distanceSensorRight = distanceSensorRight;
        this.servoDistanceSensor = servoDistanceSensor;
    }

    @NonNull
    @Override
    public String toString() {
        return " left motor power                    : " + motorLeft.getPower() + "\n" +
                "right motor power                   : " + motorRight.getPower() + "\n" +
                "left distance sensor distance (CM)  : " + distanceSensorLeft.getDistance(DistanceUnit.CM) + "\n" +
                "right distance sensor distance (CM) : " + distanceSensorRight.getDistance(DistanceUnit.CM) + "\n" +
                "distance sensor servo position      : " + servoDistanceSensor.getPosition() + "\n";
    }

    public void setDefaults() {
        // TODO: DRY

        this.motorLeft.setDirection(DcMotor.Direction.FORWARD);
        this.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.motorRight.setDirection(DcMotor.Direction.FORWARD);
        this.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ElapsedTime getElapsedTime() {
        return elapsedTime;
    }

    public DcMotor getMotorLeft() {
        return motorLeft;
    }

    public DcMotor getMotorRight() {
        return motorRight;
    }

    public DistanceSensor getDistanceSensorLeft() {
        return distanceSensorLeft;
    }

    public DistanceSensor getDistanceSensorRight() {
        return distanceSensorRight;
    }

    public Servo getServoDistanceSensor() {
        return servoDistanceSensor;
    }

    public float getCircleRotationDenominator() {
        return circleRotationDenominator;
    }

    public void setCircleRotationDenominator(float circleRotationDenominator) {
        this.circleRotationDenominator = circleRotationDenominator;
    }
}
