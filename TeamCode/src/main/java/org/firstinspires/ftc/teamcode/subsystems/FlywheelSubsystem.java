package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.apache.commons.math3.stat.StatUtils;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.OverrideCommand;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FlywheelSubsystem extends SubsystemBase {
    // hardware references
    private HardwareMap hardwareMap;
    private VoltageSensor controlHubVSensor;
    private ArrayList<DcMotorEx> flywheelMotors = new ArrayList<>();
    private DcMotorEx encoderMotor = null;

    // flywheel data
    private double countsPerFlywheelRotation;
    private double flywheelDiameterInches;

    // motor control
    private double[] voltageHistory = {12d, 12d, 12d, 12d, 12d, 12d, 12d, 12d};
    private double batteryVoltage = 12d;
    private double kS = 0.48;
    private double kV = 0.00252;
    private double pidP = 0;
    private PIDController basicPID = new PIDController(pidP, 0, 0);
    private double motorVoltage;

    // command inputs
    private final DoubleSupplier stop = () -> 0;
    private DoubleSupplier stableRPMSupplier = stop; // Commands change this to their own supplier
    private DoubleSupplier motorVoltageSupplier = stop; // Commands change this to their own supplier

    // behavior monitoring
    private double stableRPM;
    private int jamCount = 0;
    private boolean isJammed = false;
    private final int JAMMED_WHEN_COUNT_IS = 50;
    private final double JAMMED_WHEN_RPM_BELOW = 60;
    private int stableCount = 0;
    private boolean isStable = false;
    private final int STABLE_WHEN_AT_SETPOINT_COUNT = 6;
    private double stabilityTolerance = 60;

    public FlywheelSubsystem(HardwareMap hardwareMap,
                             VoltageSensor controlHubVSensor,
                             double countsPerFlywheelRotation,
                             double flywheelDiameterInches) {
        this.hardwareMap = hardwareMap;
        this.controlHubVSensor = controlHubVSensor;
        this.countsPerFlywheelRotation = countsPerFlywheelRotation;
        this.flywheelDiameterInches = flywheelDiameterInches;
        if (flywheelDiameterInches == 0) throw new IllegalArgumentException ("ASSERTION FAILED:"+
                " flywheelDiameterInches cannot be 0.");
        if (countsPerFlywheelRotation == 0) throw new IllegalArgumentException ("ASSERTION FAILED:"+
                " countsPerFlywheelRotation cannot be 0.");
        setDefaultCommand(cmdStop());
    }

    public void addFlywheelMotor(String motorName, DcMotorSimple.Direction direction) {
        try {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorName);
            motor.setDirection(direction);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (encoderMotor == null) encoderMotor = motor;
            flywheelMotors.add(motor);
        }
        catch (Exception ignored) {
            throw new IllegalArgumentException("Failed to get motor " + motorName + ".  You may" +
                    " need to power cycle the Robot.    (Did you verify the name you gave matches" +
                    " what you set in the robot configuration?)");
        }
    }

    @Override
    public void periodic() {
        System.arraycopy(voltageHistory, 0, voltageHistory, 1, voltageHistory.length - 1);
        voltageHistory[0] = controlHubVSensor.getVoltage();
        batteryVoltage = StatUtils.mean(voltageHistory);
        stableRPM = stableRPMSupplier.getAsDouble();
        motorVoltage = motorVoltageSupplier.getAsDouble();
        for (DcMotorEx flywheelMotor : flywheelMotors) {
            flywheelMotor.setPower(motorVoltage / batteryVoltage);
        }
        double measuredRPM = getMeasuredRPM();
        boolean possibleJam = (motorVoltage > kS * 2d && measuredRPM < JAMMED_WHEN_RPM_BELOW);
        boolean rpmWithinTolerance = Math.abs(measuredRPM-stableRPM) < stabilityTolerance;
        jamCount = possibleJam ? jamCount+1 : 0;
        stableCount = rpmWithinTolerance ? stableCount +1 : 0;
        isJammed = jamCount >= JAMMED_WHEN_COUNT_IS;
        isStable = stableCount >= STABLE_WHEN_AT_SETPOINT_COUNT;
    }

    private double getMeasuredRPM() {
        if (encoderMotor == null) return 0;
        return encoderMotor.getVelocity() / countsPerFlywheelRotation * 60d;
    }

    public boolean getIsJammed() {
        return isJammed;
    }

    public boolean getIsStable() {
        return isStable;
    }

    public Command cmdStop() {
        return new OverrideCommand (this){
            @Override
            public void initialize() {
                stableRPMSupplier = stop;
                motorVoltageSupplier = stop;
            }
        };
    }

    public Command cmdFindMotorConstants() {
        return new OverrideCommand(this) {
            private double requestedVoltage;
            private double lastRPM;
            private int peakCount;
            private final int TUNING_STABILITY_REQUIREMENT = 10;
            private int measurementCount;
            private double totalRPM;
            private final int SAMPLES_TO_AVERAGE = 200;
            private final SimpleRegression regression = new SimpleRegression();

            @Override
            public void initialize() {
                requestedVoltage = 2d;
                stableRPMSupplier = stop;
                motorVoltageSupplier = () -> requestedVoltage;
                totalRPM = lastRPM = peakCount = measurementCount = 0;
                regression.clear();
            }

            @Override
            public void execute() {
                double measuredRPM = getMeasuredRPM();
                if (measuredRPM < lastRPM) peakCount++;
                lastRPM = measuredRPM;
                if (peakCount < TUNING_STABILITY_REQUIREMENT) return;
                totalRPM += measuredRPM;
                measurementCount++;
                if (measurementCount < SAMPLES_TO_AVERAGE) return;
                regression.addData(totalRPM / measurementCount, requestedVoltage);
                Log.i("FTC20311", "recorded (RPM <tab> volts) = " +
                        totalRPM / measurementCount + "\t" + requestedVoltage);
                totalRPM = peakCount = measurementCount = 0;
                requestedVoltage += 1d;
            }

            @Override
            public boolean isFinished() {
                return requestedVoltage > 10;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) return;
                Log.i("FTC20311", "detected kS = " + regression.getIntercept());
                Log.i("FTC20311", "detected kV = " + regression.getSlope());
                if (kS == 0) kS = regression.getIntercept();
                if (kV == 0) kV = regression.getSlope();
                if (pidP==0) basicPID.setP(pidP = kV*8);
            }
        };
    }

    public Command cmdSetRPM(DoubleSupplier rpm, BooleanSupplier isFinished) {
        return new OverrideCommand (this) {
            @Override
            public void initialize() {
                stableRPMSupplier = () -> rpm.getAsDouble();
                motorVoltageSupplier = () -> {
                    basicPID.setSetPoint(rpm.getAsDouble());
                    double correction = basicPID.calculate(getMeasuredRPM());
                    return correction + kS + kV * rpm.getAsDouble();
                };
            }

            @Override
            public boolean isFinished() {
                return isFinished.getAsBoolean();
            }
        };
    }

    public Command cmdSetIPS(DoubleSupplier ips, BooleanSupplier isFinished) {
        return cmdSetRPM(() -> ips.getAsDouble() / Math.PI / flywheelDiameterInches * 60d, isFinished);
    }

    public Command cmdTuneWithTelemetry(double rpm) {
        Log.i("FTC20311", "Panels is located at http://192.168.43.1:8001");
        return cmdSetRPM(()->rpm, () -> {
            if(stableCount<STABLE_WHEN_AT_SETPOINT_COUNT)
                Log.i("FTC20311", "stablecount = "+stableCount);
            panelsTelemetry.addData("flywheel/measured rpm", getMeasuredRPM());
            panelsTelemetry.addData("flywheel/requested rpm", stableRPM);
            panelsTelemetry.addData("flywheel/stabilityCount", Math.min(stableCount, STABLE_WHEN_AT_SETPOINT_COUNT));
            panelsTelemetry.addData("flywheel/isStable", isStable);
            panelsTelemetry.addData("flywheel/isJammed", isJammed);
            panelsTelemetry.addData("flywheel/PID P gain", pidP);
            panelsTelemetry.update();
            return false;
        });
    }

    public Command cmdIncreaseP() {
        return new InstantCommand(() -> {
            pidP*=1.02;
            basicPID.setP(pidP);
        });
    }

    public Command cmdDecreaseP() {
        return new InstantCommand(() -> {
            pidP/=1.02;
            basicPID.setP(pidP);
        });

    }

}
