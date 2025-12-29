package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@TeleOp(name = "2025 Decode Tuning", group = "20311")
public class DecodeTuning extends CommandOpMode {
    private VoltageSensor controlHubVSensor = null;
    private FlywheelSubsystem fs;
    private GamepadEx driverGamepad;
    private GamepadButton flywheelTuneMotorConstantsButton, flywheelIdleButton;
    @Override
    public void initialize() {
        try {
            controlHubVSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        }
        catch (Exception ignored) {
            throw new IllegalArgumentException("Failed to get Control Hub voltage sensor."+
                    "  You may need to power cycle the Robot.");
        }
        fs = new FlywheelSubsystem(hardwareMap, controlHubVSensor, 28, 3);
        driverGamepad = new GamepadEx(gamepad1);
        flywheelTuneMotorConstantsButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        flywheelIdleButton = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        flywheelTuneMotorConstantsButton.whenPressed(fs.cmdTuneMotorConstants());
        flywheelIdleButton.whenPressed(fs.cmdIdle());
    }
}
