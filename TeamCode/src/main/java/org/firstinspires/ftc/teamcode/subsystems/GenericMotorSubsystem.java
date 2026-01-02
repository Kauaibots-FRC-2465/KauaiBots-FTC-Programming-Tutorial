package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public class GenericMotorSubsystem extends SubsystemBase {
    private class DcMotorExCache {
        private double lastValue;
        private int lastValueInSteps;
        private final double quantization;
        private boolean isValid=false;

        DcMotorExCache(double quantization) { this.quantization = quantization; }

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
}
