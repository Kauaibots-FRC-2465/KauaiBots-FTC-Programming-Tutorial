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

public class GenericPositioningSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    private final double countsPerRotation;
    private DcMotor.RunMode lastMode;
    private double rotationsSetPoint;
    private double positionP = 10.0d;
    private double positionPower = 0.15d;
    private double velocityP = 20.0d;
    private double velocityF = 13.9d;
    private double lastPowerModePower = Double.NaN;
    private double lastRpm =Double.NaN;
    private DcMotor.ZeroPowerBehavior defaultZeroPowerBehavior = BRAKE;
    private DcMotor.ZeroPowerBehavior lastZeroPowerBehavior = BRAKE;
    private final double rpmResolution;

    public GenericPositioningSubsystem(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction, double countsPerRotation) {
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
        motor.setZeroPowerBehavior(defaultZeroPowerBehavior);
        motor.setMode(STOP_AND_RESET_ENCODER);
        lastMode=RUN_WITHOUT_ENCODER;
        motor.setMode(lastMode);
        motor.setPower(0);
        setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
        rpmResolution = 1200.0d/countsPerRotation;  // velocity is measured in ticks per 50ms.  1 minute/50ms=1200
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (lastZeroPowerBehavior == zeroPowerBehavior) return;
        lastZeroPowerBehavior=zeroPowerBehavior;
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private void setDefaultZeroPowerBehavior(DcMotor.ZeroPowerBehavior defaultZeroPowerBehavior) {
        this.defaultZeroPowerBehavior = defaultZeroPowerBehavior;
        restoreZeroPowerBehavior();
    }

    private void restoreZeroPowerBehavior() {
        if (lastZeroPowerBehavior == defaultZeroPowerBehavior) return;
        lastZeroPowerBehavior = defaultZeroPowerBehavior;
        motor.setZeroPowerBehavior(lastZeroPowerBehavior);
    }

    private void setMotorCoefficients(double positionP, double positionPower, double velocityP, double velocityF) {
        this.positionP = positionP;
        this.positionPower = positionPower;
        this.velocityP = velocityP;
        this.velocityF = velocityF;
        if(lastMode==RUN_TO_POSITION) motor.setPower(positionPower);
        motor.setPositionPIDFCoefficients(positionP);
        motor.setVelocityPIDFCoefficients(lastMode==RUN_USING_ENCODER?velocityP:0, 0, 0, velocityF);
    }

    private void setPower(double power) {
        if (Math.abs(power - lastPowerModePower)<.001 && !Double.isNaN(lastPowerModePower)) return;
        lastPowerModePower = power;
        motor.setPower(power);
    }

    private void goPower() {
        restoreZeroPowerBehavior();
        if (lastMode != RUN_WITHOUT_ENCODER) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            lastMode = RUN_WITHOUT_ENCODER;
            lastRpm = Double.NaN;
        }
    }

    private void goRPM() {
        restoreZeroPowerBehavior();
        if (lastMode != RUN_USING_ENCODER)
        {
            motor.setMode(RUN_USING_ENCODER);
            lastMode = RUN_USING_ENCODER;
            setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
            lastPowerModePower = Double.NaN;
        }
    }

    private void setRPM(double rpm) {
        if (Math.abs(rpm-lastRpm)<rpmResolution/2 && !Double.isNaN(lastRpm)) return;
        lastRpm = rpm;
        motor.setVelocity(rpm * countsPerRotation / 60.0d);
    }

    private double getMeasuredRotations() {
        return (double)motor.getCurrentPosition()/ countsPerRotation;
    }

    private double getMeasuredRPM()
    {
        return motor.getVelocity() / countsPerRotation * 60.0;
    }


    private void moveTo(double rotations) {
        restoreZeroPowerBehavior();
        rotationsSetPoint = rotations;
        motor.setTargetPosition((int) Math.round(rotationsSetPoint * countsPerRotation));
        if (lastMode != RUN_TO_POSITION) {
            motor.setMode(RUN_TO_POSITION);
            lastMode = RUN_TO_POSITION;
            setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
            lastPowerModePower = Double.NaN;
            lastRpm = Double.NaN;
        }
    }

    public Command cmdSetDefaultZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        return new InstantCommand(() -> setDefaultZeroPowerBehavior(zeroPowerBehavior));
    }

    public Command cmdSetPower(DoubleSupplier power) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
                goPower();
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
                goRPM();
            }

            @Override
            public void execute() {
                setRPM(rpm.getAsDouble());
            }
        };
    }

    public Command cmdWaitUntilInPosition(double rotationsTolerance) {
        return new WaitUntilCommand
                (() -> Math.abs((getMeasuredRotations() - rotationsSetPoint)) < rotationsTolerance);
    }

    private Command cmdMoveTo(DoubleSupplier rotations) {
        return new OverrideCommand(this) {
            @Override
            public void initialize() {
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

    private Command cmdChangePositionPower(double ammount) {
        return new InstantCommand(() -> {
            positionPower = MathUtils.clamp(positionPower+ammount, 0, 1);
            Log.i("FTC20311", "Position power = "+positionPower);
            setMotorCoefficients(positionP, positionPower, velocityP, velocityF);
        });
    }

    private Command cmdChangeVelocityP(double scale) {
        return new InstantCommand(() -> setMotorCoefficients(positionP, positionPower, velocityP*scale, velocityF));
    }

    public Command cmdStop() { return cmdSetPower(()->0); }
    public Command cmdBrake() { return cmdBrakeOrFloat(BRAKE); }
    public Command cmdFloat() { return cmdBrakeOrFloat(FLOAT); }
    public Command cmdAdvanceFromHere(double rotations) { return cmdMoveTo (() -> getMeasuredRotations() + rotations); }
    public Command cmdAdvanceAdditional(double rotations) { return cmdMoveTo (() -> rotationsSetPoint + rotations); }
    public Command cmdRegressFromHere(double rotations) { return cmdMoveTo (() -> getMeasuredRotations() - rotations); }
    public Command cmdRegressAdditional(double rotations) { return cmdMoveTo (() -> rotationsSetPoint - rotations); }
    public Command cmdIncreasePositionP() { return cmdChangePositionP(1.02); }
    public Command cmdDecreasePositionP() { return cmdChangePositionP(1.0/1.02); }
    public Command cmdIncreasePositionPower() { return cmdChangePositionPower(.05); }
    public Command cmdDecreasePositionPower() { return cmdChangePositionPower(-.05); }
    public Command cmdIncreaseVelocityP() { return cmdChangeVelocityP(1.02); }
    public Command cmdDecreaseVelocityP() { return cmdChangeVelocityP(1.0/1.02); }

    public Command cmdFindMotorConstants(DoubleSupplier RPM) {
        return cmdSetRPM(RPM).alongWith(new SequentialCommandGroup(
                new InstantCommand( ()-> {
                    if(velocityF == 0) velocityF = 10.0d;
                    Log.i("FTC20311", "Requested RPM = " + RPM.getAsDouble());
                    Log.i("FTC20311", "Expect RPM steps of = " + rpmResolution);
                    Log.i("FTC20311", "Initial velocityF = " + velocityF);
                    setMotorCoefficients(positionP, positionPower, 0, velocityF);
                    Log.i("FTC20311", "a ");
                }),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "b ");
                }),
                new WaitCommand(2000),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "c");
                }),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "Measured RPM = " + getMeasuredRPM());
                    setMotorCoefficients(positionP,positionPower, 0, velocityF*RPM.getAsDouble()/ getMeasuredRPM());
                }),
                new WaitCommand(2000),
                new InstantCommand( ()-> {
                    Log.i("FTC20311", "New F measured RPM = " + getMeasuredRPM());
                    setMotorCoefficients(positionP, positionPower, velocityF*2d, velocityF);
                    Log.i("FTC20311", "New P+F measured RPM = " + getMeasuredRPM());
                    Log.i("FTC20311", "Suggested velocityP = " + velocityP);
                    Log.i("FTC20311", "Calculated velocityF = " + velocityF);
                }),
                new WaitCommand(1000),
                new RepeatCommand(
                        new WaitCommand(100).andThen(
                                new InstantCommand( ()->
                                        Log.i("FTC20311",
                                                String.format(
                                                        "VP=%.2f VF=%.2f RPM=%.2f WANT=%.2f",
                                                        velocityP,
                                                        velocityF,
                                                        getMeasuredRPM(),
                                                        RPM.getAsDouble()
                                                )
                                        )
                                )
                        )
                )
        ));
    }

    public Command cmdTunePositionP() {
        return cmdAdvanceFromHere(0);
    }
}
