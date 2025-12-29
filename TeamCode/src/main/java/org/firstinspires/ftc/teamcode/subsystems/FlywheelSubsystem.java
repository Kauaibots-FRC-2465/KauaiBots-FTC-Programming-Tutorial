package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.apache.commons.math3.stat.StatUtils;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.OverrideCommand;

import java.util.ArrayList;
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
    private final DoubleSupplier idle = () -> 0;
    private DoubleSupplier motorVoltageSupplier = idle; // Commands must provide their own supplier
    private double motorVoltage;
    private double kS = 0;
    private double kV = 0;

    // Behavior Monitoring
    private int jamCount = 0;
    private boolean isJammed = false;
    private final int JAMMED_WHEN_COUNT_IS = 50;
    private final double JAMMED_WHEN_RPM_BELOW = 60;
    private int stabilityCount;

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
        setDefaultCommand(cmdIdle());
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
        motorVoltage = motorVoltageSupplier.getAsDouble();
        for (DcMotorEx flywheelMotor : flywheelMotors) {
            flywheelMotor.setPower(motorVoltage / batteryVoltage);
        }
        boolean possibleJam = (motorVoltage > kS * 2d && getCurrentRPM() < JAMMED_WHEN_RPM_BELOW);
        jamCount = possibleJam ? jamCount+1 : 0;
        isJammed = jamCount >= JAMMED_WHEN_COUNT_IS;
    }

    private double getCurrentRPM() {
        if (encoderMotor == null) return 0;
        return encoderMotor.getVelocity() / countsPerFlywheelRotation * 60d;
    }

    public Command cmdIdle() {
        return new OverrideCommand (this){
            @Override
            public void initialize() {
                motorVoltageSupplier = idle;
            }
        };
    }

    public Command cmdTuneMotorConstants() {
        return new OverrideCommand(this) {
            private double requestedVoltage;

            private double lastRPM;
            private final int TUNING_STABILITY_REQUIREMENT = 10;
            private int measurementCount;
            private double totalRPM;
            private final int SAMPLES_TO_AVERAGE = 200;
            private final SimpleRegression regression = new SimpleRegression();

            @Override
            public void initialize() {
                requestedVoltage = 2d;
                motorVoltageSupplier = () -> requestedVoltage;
                totalRPM = lastRPM = stabilityCount = measurementCount = 0;
                regression.clear();
            }

            @Override
            public void execute() {
                double currentRPM = getCurrentRPM();
                if (currentRPM < lastRPM) stabilityCount++;
                lastRPM = currentRPM;
                if (stabilityCount < TUNING_STABILITY_REQUIREMENT) return;
                totalRPM += getCurrentRPM();
                measurementCount++;
                if (measurementCount < SAMPLES_TO_AVERAGE) return;
                regression.addData(totalRPM / measurementCount, requestedVoltage);
                stabilityCount = measurementCount = 0;
                totalRPM = 0;
                requestedVoltage += 1d;
            }

            @Override
            public boolean isFinished() {
                return requestedVoltage > 10;
            }

            @Override
            public void end(boolean interrupted) {
                kS = regression.getIntercept();
                kV = regression.getSlope();
                Log.i("FTC20311", "detected kS = " + kS);
                Log.i("FTC20311", "detected kV = " + kV);
            }
        };
    }
}
