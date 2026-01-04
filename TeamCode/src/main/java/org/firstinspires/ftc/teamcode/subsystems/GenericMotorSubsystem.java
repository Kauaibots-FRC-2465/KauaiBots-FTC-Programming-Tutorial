package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.OverrideCommand;

import java.util.function.DoubleSupplier;

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

    // RUN_USING_ENCODER cache - hardware quantizes to 20 ticks/sec
    private final DcMotorExCache lastRUEcps = new DcMotorExCache(20d);

    // RUN_WITHOUT_ENCODER cache - hardware uses a 16-bit signed integer (32767 steps)
    private final DcMotorExCache lastRWEpower = new DcMotorExCache(1d/32767d);

    // RUN_TO_POSITION cache - hardware quantizes to individual encoder ticks
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

    private void setDefaultZeroPowerBehavior(DcMotor.ZeroPowerBehavior defaultZeroPowerBehavior) {
        this.defaultZPB = defaultZeroPowerBehavior;
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (lastZPB == zeroPowerBehavior) return;
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        lastZPB = zeroPowerBehavior;
    }

    private void restoreZeroPowerBehavior() {
        setZeroPowerBehavior(defaultZPB);
    }

    private void setMotorCoefficients(double positionP, double positionPower, double velocityP, double velocityF) {
        this.positionP = positionP;
        this.positionPower = positionPower;
        this.velocityP = velocityP;
        this.velocityF = velocityF;
        if (lastRunMode == RUN_TO_POSITION) {
            motor.setPower(positionPower);
            motor.setPositionPIDFCoefficients(positionP);
        }
        if (lastRunMode == RUN_WITHOUT_ENCODER) return;
        motor.setVelocityPIDFCoefficients(lastRunMode ==RUN_USING_ENCODER?velocityP:0, 0, 0, velocityF);
    }

    /**
     * Switch the mode of the motor
     * Warning: remember to moveTo() before you switchModes(), or the motor may jerk
     */
    private void switchModes(DcMotor.RunMode runMode)
    {
        restoreZeroPowerBehavior();
        if (lastRunMode == runMode) return;
        lastRunMode = runMode;
        if(runMode == RUN_TO_POSITION) motor.setTargetPosition(lastRTPcounts.getAsInt());
        motor.setMode(runMode);
        setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
        if(runMode != RUN_WITHOUT_ENCODER) lastRWEpower.invalidate();
        if(runMode != RUN_USING_ENCODER) lastRUEcps.invalidate();
        if(runMode != RUN_TO_POSITION) lastRTPcounts.invalidate();
    }

    private double getMeasuredRotations() {
        return (double)motor.getCurrentPosition() / countsPerRotation;
    }

    private double getMeasuredRPM()
    {
        return motor.getVelocity() / countsPerRotation * 60.0;
    }

    private void setPower(double power) {
        if (lastRWEpower.cacheAndGate(power)) {
            motor.setPower(power);
        }
    }

    private void setRPM(double rpm) {
        if (lastRUEcps.cacheAndGate(rpm * countsPerRotation / 60.0d)) {
            motor.setVelocity(lastRUEcps.get());
        }
    }

    private void moveTo(double rotations) {
        if (lastRTPcounts.cacheAndGate(rotations * countsPerRotation))
            motor.setTargetPosition(lastRTPcounts.getAsInt());
    }

    public Command cmdSetPower(DoubleSupplier power) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                switchModes(RUN_WITHOUT_ENCODER);
            }

            @Override
            public void execute() {
                setPower(power.getAsDouble());
            }
        };
    }

    public Command cmdSetRPM(DoubleSupplier rpm) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                switchModes(RUN_USING_ENCODER);
            }

            @Override
            public void execute() {
                setRPM(rpm.getAsDouble());
            }
        };
    }

    private Command cmdMoveTo(DoubleSupplier staticRotations, DoubleSupplier dynamicRotations, Double tolerance) {
        return new OverrideCommand(this) {
            double offset;
            @Override
            public void initialize() {
                offset=staticRotations.getAsDouble();
                // extra moveTo required to prevent the motor from jerking when switching modes
                moveTo(offset+dynamicRotations.getAsDouble());
                switchModes(RUN_TO_POSITION);
            }

            @Override
            public void execute() {
                moveTo(offset+dynamicRotations.getAsDouble());
            }

            @Override
            public boolean isFinished() {
                if (tolerance==null) return false;
                return Math.abs(getMeasuredRotations() - (offset+dynamicRotations.getAsDouble())) < tolerance;
            }
        };
    }

    // Brakes or floats immediately, but does not change default behavior
    private Command cmdBrakeOrFloat(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                setZeroPowerBehavior(zeroPowerBehavior);
                switchModes(RUN_WITHOUT_ENCODER);
                setPower(0);
            }
        };
    }

    private Command cmdChangePositionP(double scale) {
        return new InstantCommand(() -> {
            setMotorCoefficients(positionP*scale, positionPower, velocityP, velocityF);
            Log.i("FTC20311", "Position p = "+positionP);
        });
    }

    private Command cmdChangePositionPower(double amount) {
        return new InstantCommand(() -> {
            positionPower = MathUtils.clamp(positionPower+amount, 0, 1);
            setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
            Log.i("FTC20311", "Position power = "+positionPower);
        });
    }

    private Command cmdChangeVelocityP(double scale) {
        return new InstantCommand(() -> setMotorCoefficients(positionP, positionPower, velocityP*scale, velocityF));
    }

    public Command cmdBrake() { return cmdBrakeOrFloat(BRAKE); }
    public Command cmdFloat() { return cmdBrakeOrFloat(FLOAT); }
    public Command cmdMoveTo(DoubleSupplier rotations) { return cmdMoveTo(()->0, rotations, null); }
    public Command cmdAdvanceFromHere(DoubleSupplier rotations) { return cmdMoveTo (() -> getMeasuredRotations(), () -> rotations.getAsDouble(), null ); }
    public Command cmdAdvanceFromSetpoint(DoubleSupplier rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations(), () -> rotations.getAsDouble(), null); }
    public Command cmdRegressFromHere(DoubleSupplier rotations) { return cmdMoveTo (() -> getMeasuredRotations(), () -> -rotations.getAsDouble(), null); }
    public Command cmdRegressFromSetpoint(DoubleSupplier rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations(),() -> -rotations.getAsDouble(), null); }
    public Command cmdMoveToAndWait(DoubleSupplier rotations, double tolerance) { return cmdMoveTo(()->0, rotations, tolerance); }
    public Command cmdAdvanceFromHereAndWait(DoubleSupplier rotations, double tolerance) { return cmdMoveTo (()->getMeasuredRotations(), rotations, tolerance); }
    public Command cmdAdvanceFromSetpointAndWait(DoubleSupplier rotations, double tolerance) { return cmdMoveTo (()->lastRTPcounts.getRotations(), rotations, tolerance); }
    public Command cmdRegressFromHereAndWait(DoubleSupplier rotations, double tolerance) { return cmdMoveTo (()->getMeasuredRotations(), ()->-rotations.getAsDouble(), tolerance); }
    public Command cmdRegressFromSetpointAndWait(DoubleSupplier rotations, double tolerance) { return cmdMoveTo (()->lastRTPcounts.getRotations(), ()->-rotations.getAsDouble(), tolerance); }
    public Command cmdMoveTo(double rotations) { return cmdMoveTo(()->0,()->rotations, null); }
    public Command cmdAdvanceFromHere(double rotations) { return cmdMoveTo (() -> getMeasuredRotations(), () -> rotations, null ); }
    public Command cmdAdvanceFromSetpoint(double rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations(), () -> rotations, null); }
    public Command cmdRegressFromHere(double rotations) { return cmdMoveTo (() -> getMeasuredRotations(), () -> -rotations, null); }
    public Command cmdRegressFromSetpoint(double rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations(),() -> -rotations, null); }
    public Command cmdMoveToAndWait(double rotations, double tolerance) { return cmdMoveTo(()->0, ()->rotations, tolerance); }
    public Command cmdAdvanceFromHereAndWait(double rotations, double tolerance) { return cmdMoveTo (()->getMeasuredRotations(), ()->rotations, tolerance); }
    public Command cmdAdvanceFromSetpointAndWait(double rotations, double tolerance) { return cmdMoveTo (()->lastRTPcounts.getRotations(), ()->rotations, tolerance); }
    public Command cmdRegressFromHereAndWait(double rotations, double tolerance) { return cmdMoveTo (()->getMeasuredRotations(), ()->-rotations, tolerance); }
    public Command cmdRegressFromSetpointAndWait(double rotations, double tolerance) { return cmdMoveTo (()->lastRTPcounts.getRotations(), ()->-rotations, tolerance); }
    public Command cmdIncreasePositionP() { return cmdChangePositionP(1.02); }
    public Command cmdDecreasePositionP() { return cmdChangePositionP(1.0/1.02); }
    public Command cmdIncreasePositionPower() { return cmdChangePositionPower(.05); }
    public Command cmdDecreasePositionPower() { return cmdChangePositionPower(-.05); }
    public Command cmdIncreaseVelocityP() { return cmdChangeVelocityP(1.02); }
    public Command cmdDecreaseVelocityP() { return cmdChangeVelocityP(1.0/1.02); }

    public Command cmdFindMotorConstants(DoubleSupplier RPM) {
        Runnable report = ()->
                Log.i("FTC20311",String.format(
                        "VP=%.2f VF=%.2f RPM=%.2f WANT=%.2f",
                        velocityP,
                        velocityF,
                        getMeasuredRPM(),
                        RPM.getAsDouble()));
        Command keepReporting = new RepeatCommand(
                new WaitCommand(100)
                        .andThen(new InstantCommand(report)));
        return cmdSetRPM(RPM).alongWith(new SequentialCommandGroup(
                new InstantCommand( ()-> {
                    if(velocityF == 0) velocityF = 10.0d;
                    Log.i("FTC20311", "Target RPM = " + RPM.getAsDouble());
                    Log.i("FTC20311", "RPM quantization = " + 1200.0d/countsPerRotation);
                    Log.i("FTC20311", "Initial velocityF = " + velocityF);
                    setMotorCoefficients(positionP, positionPower, 0, velocityF);
                }),
                new WaitCommand(2000),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "Measured RPM with no P = " + getMeasuredRPM());
                    velocityF *= RPM.getAsDouble()/ getMeasuredRPM();
                    Log.i("FTC20311", "Setting velocityF = " + velocityF);
                    setMotorCoefficients(positionP,positionPower, 0, velocityF);
                }),
                new WaitCommand(2000),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "Measured ROM with no P = " + getMeasuredRPM());
                    setMotorCoefficients(positionP, positionPower, velocityF*2d, velocityF);
                    Log.i("FTC20311", "Setting velocityP = " + velocityP);
                }),
                new WaitCommand(2000),
                keepReporting
        ));
    }

    public Command cmdTunePositionP() {
        return cmdAdvanceFromHere(()->0);
    }
}