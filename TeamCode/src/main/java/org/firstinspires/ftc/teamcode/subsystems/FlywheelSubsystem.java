package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.apache.commons.math3.stat.StatUtils;
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
    private double kS = 0;
    private double kV = 0;
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
}
