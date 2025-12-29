package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;

@TeleOp(name = "2025 Decode Tuning", group = "20311")
public class DecodeTuning extends CommandOpMode {
    private FlywheelSubsystem fs;
    private VoltageSensor controlHubVSensor = null;
    private GamepadEx driverGamepad;
    private GamepadButton
            findMotorConstantsButton,
            idleButton;
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
        fs.addFlywheelMotor("shooter1", DcMotorSimple.Direction.REVERSE);
        fs.addFlywheelMotor("shooter2", DcMotorSimple.Direction.REVERSE);
        driverGamepad = new GamepadEx(gamepad1);
        findMotorConstantsButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CROSS); // aka A;
        idleButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CIRCLE); // aka B
        findMotorConstantsButton.whenPressed(fs.cmdFindMotorConstants());
        idleButton.whenPressed(fs.cmdStop());
    }
}
