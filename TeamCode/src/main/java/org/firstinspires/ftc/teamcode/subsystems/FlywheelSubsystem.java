package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.stat.StatUtils;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.firstinspires.ftc.teamcode.OverrideCommand;

import java.util.ArrayList;
import java.util.Arrays;
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
    private final DoubleSupplier idle = () -> 0;
    private DoubleSupplier motorVoltageSupplier; // Commands must provide their own supplier
    private double motorVoltage;
    private final double kS = 0.48; // How many volt needed to break static friction
    private final double kV = 0.00252; // How many volts for each RPM
    private double pidP = kV*8; // An initial guess, you want to check with tuning
    private PIDController basicPID = new PIDController(pidP, 0, 0);

    // Behavior Monitoring
    private int jamCount = 0;
    private boolean isJammed = false;
    private final int JAMMED_WHEN_COUNT_IS = 50;
    private final double JAMMED_WHEN_RPM_BELOW = 60;
    private boolean launchStarted;
    private boolean isStable;
    private boolean wasStable;
    private boolean launchEnded;
    private int stabilityCount;
    private double lastVelocity;
    private int launchVelocityRiseCount;
    private final int STABLE_WHEN_AT_SETPOINT_COUNT = 5;
    private final double SHOT_STARTS_WHEN_RPM_DROPS = 200;
    private final int LAUNCH_COMPLETE_WHEN_VELOCITY_RISE_COUNT = 5;

    // Telemetry
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

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
                lastRPM = 0;
                stabilityCount = 0;
                measurementCount = 0;
                totalRPM = 0;
                regression.clear();

                Log.i("FTC20311", "running CmdTuneKs");
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
                stabilityCount = 0;
                measurementCount = 0;
                totalRPM = 0;
                requestedVoltage += 1d;
            }

            @Override
            public boolean isFinished() {
                return requestedVoltage > 10;
            }

            @Override
            public void end(boolean interrupted) {
                Log.i("FTC20311", "detected kS = " + regression.getIntercept());
                Log.i("FTC20311", "detected kV = " + regression.getSlope());
            }
        };
    }


    public Command cmdSetRPM(DoubleSupplier rpm, BooleanSupplier isFinished) {
        return new FunctionalCommand(
                () -> {
                    basicPID.setTolerance(60);
                    motorVoltageSupplier = () -> {
                        basicPID.setSetPoint(rpm.getAsDouble());
                        double correction = basicPID.calculate(getCurrentRPM());
                        return correction + kS + kV * rpm.getAsDouble();
                    };
                    isStable = false;
                    wasStable = false;
                    launchStarted = false;
                    launchEnded = false;
                    launchVelocityRiseCount = 0;
                    stabilityCount = 0;
                },
                () -> {
                    double currentRPM = getCurrentRPM();
                    if (basicPID.atSetPoint()) ++stabilityCount; else stabilityCount = 0;
                    isStable = stabilityCount >= STABLE_WHEN_AT_SETPOINT_COUNT;
                    if (isStable) wasStable = true;
                    if (currentRPM <= (rpm.getAsDouble() - SHOT_STARTS_WHEN_RPM_DROPS) && wasStable) {
                        if (launchStarted && currentRPM > lastVelocity)
                            launchVelocityRiseCount++;
                        if (launchVelocityRiseCount == LAUNCH_COMPLETE_WHEN_VELOCITY_RISE_COUNT)
                            launchEnded = true;
                        launchStarted = true;
                        lastVelocity = currentRPM;
                    }
                },
                (Boolean interrupted) -> {},
                isFinished,
                this);
    }

    public Command cmdSetIPS(DoubleSupplier ips, BooleanSupplier isFinished) {
        return cmdSetRPM(() -> ips.getAsDouble() / Math.PI / flywheelDiameterInches * 60d, isFinished);
    }

    public Command cmdTuneWithTelemetry(DoubleSupplier rpm, BooleanSupplier isFinished) {
        Log.i("FTC20311", "Panels is located at http://192.168.43.1:8001");
        return cmdSetRPM(rpm, () -> {
            panelsTelemetry.addData("flywheel/measured rpm", getCurrentRPM());
            panelsTelemetry.addData("flywheel/requested rpm", rpm.getAsDouble());
            panelsTelemetry.addData("flywheel/stabilityCount", Math.min(stabilityCount, STABLE_WHEN_AT_SETPOINT_COUNT));
            panelsTelemetry.addData("flywheel/isStable", isStable);
            panelsTelemetry.addData("flywheel/isJammed", isJammed);
            panelsTelemetry.addData("flywheel/launchStarted", launchStarted);
            panelsTelemetry.addData("flywheel/launchEnded", launchEnded);
            panelsTelemetry.addData("flywheel/PID P gain", pidP);
            panelsTelemetry.update();
            return isFinished.getAsBoolean();
        });
    }

    public Command cmdIncreaseP(double deltaP) {
        return new InstantCommand(() -> {
            pidP+=Math.abs(deltaP);
            basicPID.setP(pidP);
        });
    }

    public Command cmdDecreaseP(double deltaP) {
        return new InstantCommand(() -> {
            pidP-=Math.abs(deltaP);
            basicPID.setP(pidP);
        });
    }

    public Command cmdWaitForFlywheelStable() {
        return new WaitUntilCommand(() -> isStable);
    }

    public Command cmdWaitForLaunchStart() {
        return new WaitUntilCommand(() -> launchStarted);
    }

    public Command cmdWaitForLaunchEnd() {
        return new WaitUntilCommand(() -> {
            if (launchEnded) {
                wasStable = false;
                launchStarted = false;
                launchEnded = false;
                launchVelocityRiseCount = 0;
                stabilityCount = 0;
                return true;
            } else return false;
        });
    }

    private final double UNJAM_VOLTAGE = 0.2d*12d;

    public Command cmdUnjam() {
        return new OverrideCommand(this) {
            ElapsedTime elapsedTime = new ElapsedTime();

            @Override
            public void initialize() {
                elapsedTime.reset();
                motorVoltageSupplier = () -> (int) elapsedTime.seconds() % 2 == 0 ? -UNJAM_VOLTAGE : UNJAM_VOLTAGE;
            }
        };
    }

    public boolean getIsJammed() {
        return isJammed;
    }
}
