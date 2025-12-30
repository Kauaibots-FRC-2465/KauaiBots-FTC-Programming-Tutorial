package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
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
    private GamepadButton findMotorConstantsButton;
    private GamepadButton stopButton;
    private GamepadButton tuneButton;
    private GamepadButton increasePButton;
    private GamepadButton decreasePButton;
    private GamepadButton testLaunchDetectionButton;
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
        stopButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CIRCLE); // aka B
        tuneButton = new GamepadButton(driverGamepad, GamepadKeys.Button.SQUARE); // aka X
        increasePButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        decreasePButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        testLaunchDetectionButton = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER); // aka B
        findMotorConstantsButton.whenPressed(fs.cmdFindMotorConstants());
        stopButton.whenPressed(fs.cmdStop());
        tuneButton.whenHeld(fs.cmdTuneWithTelemetry(1500d));
        increasePButton.whenPressed(fs.cmdIncreaseP());
        decreasePButton.whenPressed(fs.cmdDecreaseP());
        testLaunchDetectionButton.whenPressed(
            fs.cmdSetRPM(()->1500, ()->false).raceWith(
                new WaitCommand(50).andThen(
                fs.cmdWaitUntilStable(),
                cmdLog("Stable "+System.nanoTime()),
                fs.cmdWaitLaunchStart(1500, 200),
                cmdLog("Launch started at "+System.nanoTime()),
                fs.cmdWaitLaunchEnd(5),
                cmdLog("Launch ended at "+System.nanoTime()))
            ));
    }

    public Command cmdLog (String info) {
        return new InstantCommand(()-> Log.i("FTC20311", info));
    }

}
