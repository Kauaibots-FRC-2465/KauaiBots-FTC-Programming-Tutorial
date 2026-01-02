package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class GenericMotorSubsystem extends SubsystemBase {
    private class DcMotorExCache {
        private double lastValue;
        private int lastValueInSteps;
        private final double quantization;
        private boolean isValid=false;

        private DcMotorExCache(double quantization) { this.quantization = quantization; }

        void invalidate() { isValid = false; }

        boolean cacheAndGate(double newValue)
        {
            int valueInSteps = (int) Math.round(newValue/quantization);
            lastValue = newValue;
            if (lastValueInSteps == valueInSteps && isValid) return false;
            isValid = true;
            lastValueInSteps = valueInSteps;
            return true;
        }

        double get() { return lastValue; }
        int getAsInt() { return lastValueInSteps; }
        double getRotations() { return lastValue/countsPerRotation; }
    }

    private double countsPerRotation;
    private final DcMotorEx motor;
    // Default values
    private double positionP = 10d;
    private double positionPower = 1d;
    private double velocityP = 0d;
    private double velocityF = 0d;
    private DcMotor.ZeroPowerBehavior defaultZPB = BRAKE;
    // RunMode cache
    private DcMotor.RunMode lastRunMode;
    // RUN_USING_ENCODER cache
    private final DcMotorExCache lastRUEcps  = new DcMotorExCache(20d);
    // RUN_WITHOUT_ENCODER cache
    private final DcMotorExCache lastRWEpower = new DcMotorExCache(1d/32767d);
    // RUN_TO_POSITION cache
    private final DcMotorExCache lastRTPcounts = new DcMotorExCache(1d);
    // ZeroPowerBehavior cache
    private DcMotor.ZeroPowerBehavior lastZPB;

    public GenericMotorSubsystem(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction, double countsPerRotation) {
        this.countsPerRotation = countsPerRotation;
        try {
            motor = hardwareMap.get(DcMotorEx.class, motorName);
        }
        catch (Exception ignored) {
            throw new IllegalArgumentException("Failed to get motor " + motorName + ".  You may" +
                    " need to power cycle the Robot.    (Did you verify the name you gave matches" +
                    " what you set in the robot configuration?)");
        }
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(defaultZPB);
        lastZPB = defaultZPB;
        motor.setMode(STOP_AND_RESET_ENCODER);
        lastRunMode = STOP_AND_RESET_ENCODER;
    }
}