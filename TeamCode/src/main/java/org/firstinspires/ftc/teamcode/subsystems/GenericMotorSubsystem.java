package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
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
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.OverrideCommand;

import java.util.function.DoubleSupplier;

public class GenericMotorSubsystem extends SubsystemBase {
    private class Cache {
        private double lastValue;
        private int lastValueInSteps;
        private final double quantization;
        private boolean isValid=false;

        Cache(double quantization) { this.quantization = quantization; }

        void invalidate() { isValid = false; }

        boolean hasChanged(double newValue)
        {
            int valueInSteps = (int) Math.round(newValue/quantization);
            if (lastValueInSteps == valueInSteps && isValid) return false;
            isValid = true;
            lastValue = newValue;
            lastValueInSteps = valueInSteps;
            return true;
        }

        int getAsInt() { return lastValueInSteps; }

        double getRotations() { return lastValueInSteps/countsPerRotation; }

        double get() { return lastValue; }
    }

    private final DcMotorEx motor;
    private final double countsPerRotation;
    // Default values
    private double positionP = 10.0d;
    private double positionPower = 0.15d;
    private double velocityP = 20.0d;
    private double velocityF = 13.9d;
    private DcMotor.ZeroPowerBehavior defaultZPB = BRAKE;
    // RunMode cache
    private DcMotor.RunMode lastRunMode;
    // RUN_USING_ENCODER cache
    private final Cache lastRUEcps  = new Cache(20d);
    // RUN_WITHOUT_ENCODER cache
    private final Cache lastRWEpower = new Cache(1d/32767d);
    // RUN_TO_POSITION cache
    private final Cache lastRTPcounts = new Cache(1d);
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
        if(lastRunMode == RUN_TO_POSITION) motor.setPower(positionPower);
        motor.setPositionPIDFCoefficients(positionP);
        motor.setVelocityPIDFCoefficients(lastRunMode ==RUN_USING_ENCODER?velocityP:0, 0, 0, velocityF);
    }

    private void setPower(double power) {
        if (lastRWEpower.hasChanged(power)) motor.setPower(power);
    }

    private void goRWE() {
        restoreZeroPowerBehavior();
        if (lastRunMode == RUN_WITHOUT_ENCODER) return;
        motor.setMode(RUN_WITHOUT_ENCODER);
        lastRunMode = RUN_WITHOUT_ENCODER;
        lastRUEcps.invalidate();
        lastRTPcounts.invalidate();
    }

    private void goRUE() {
        restoreZeroPowerBehavior();
        if (lastRunMode == RUN_USING_ENCODER) return;
        lastRunMode = RUN_USING_ENCODER;
        motor.setMode(lastRunMode);
        setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
        lastRWEpower.invalidate();
        lastRTPcounts.invalidate();
    }

    private void goRTP(double rotations) {
        restoreZeroPowerBehavior();
        if (lastRunMode == RUN_TO_POSITION) return;
        lastRTPcounts.hasChanged(rotations * countsPerRotation);
        motor.setTargetPosition(lastRTPcounts.getAsInt());
        motor.setMode(RUN_TO_POSITION);
        lastRunMode = RUN_TO_POSITION;
        setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
        lastRWEpower.invalidate();
        lastRUEcps.invalidate();
    }

    private double getMeasuredRotations() {
        return (double)motor.getCurrentPosition() / countsPerRotation;
    }

    private double getMeasuredRPM()
    {
        return motor.getVelocity() / countsPerRotation * 60.0;
    }

    private void setRPM(double rpm) {
        if (lastRUEcps.hasChanged (rpm * countsPerRotation / 60.0d)) {
            motor.setVelocity(lastRUEcps.get());
        }
    }

    private void moveTo(double rotations) {
        if (lastRTPcounts.hasChanged(rotations * countsPerRotation))
            motor.setTargetPosition(lastRTPcounts.getAsInt());
    }

    public Command cmdSetDefaultZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        return new InstantCommand(() -> setDefaultZeroPowerBehavior(zeroPowerBehavior));
    }

    public Command cmdSetPower(DoubleSupplier power) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                goRWE();
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
                goRUE();
            }

            @Override
            public void execute() {
                setRPM(rpm.getAsDouble());
            }
        };
    }

    public Command cmdWaitUntilInPosition(double rotationsTolerance) {
        return new WaitUntilCommand
                (() -> Math.abs((getMeasuredRotations() - lastRTPcounts.getRotations())) < rotationsTolerance);
    }

    private Command cmdMoveTo(DoubleSupplier rotations) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                goRTP(rotations.getAsDouble());
            }

            @Override
            public void execute() {
                moveTo(rotations.getAsDouble());
            }
        };
    }

    // Brakes or floats immediately, but does not change default behavior
    private Command cmdBrakeOrFloat(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                setZeroPowerBehavior(zeroPowerBehavior);
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

    public Command cmdStop() { return cmdSetPower(()->0); }
    public Command cmdBrake() { return cmdBrakeOrFloat(BRAKE); }
    public Command cmdFloat() { return cmdBrakeOrFloat(FLOAT); }
    public Command cmdAdvanceFromHere(DoubleSupplier rotations) { return cmdMoveTo (() -> getMeasuredRotations() + rotations.getAsDouble()); }
    public Command cmdAdvanceAdditional(DoubleSupplier rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations() + rotations.getAsDouble()); }
    public Command cmdRegressFromHere(DoubleSupplier rotations) { return cmdMoveTo (() -> getMeasuredRotations() - rotations.getAsDouble()); }
    public Command cmdRegressAdditional(DoubleSupplier rotations) { return cmdMoveTo (() -> lastRTPcounts.getRotations() - rotations.getAsDouble()); }
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
