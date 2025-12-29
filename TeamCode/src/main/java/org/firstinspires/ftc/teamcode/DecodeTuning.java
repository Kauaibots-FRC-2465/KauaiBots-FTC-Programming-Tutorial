package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@TeleOp(name = "2025 Decode Tuning", group = "20311")
public class DecodeTuning extends CommandOpMode {
    private GamepadEx driverGamepad;
    private GamepadButton tuneMotorConstantsButton;
    private FlywheelSubsystem fs;
    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        tuneMotorConstantsButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);

        tuneMotorConstantsButton.whenPressed(fs.cmdTuneMotorConstants());
    }
}
