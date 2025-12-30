package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroPathingSubsystem;

import java.util.function.Supplier;

@TeleOp(name = "2025 Decode TeleOp", group = "20311")
public class DecodeTeleop extends CommandOpMode {
    private PedroPathingSubsystem pps;
    private GamepadEx driverGamepad, engineerGamepad;
    private GamepadButton reorientButton;

    @Override
    public void initialize() {
        pps = new PedroPathingSubsystem(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);
        engineerGamepad = new GamepadEx(gamepad2);
        reorientButton = new GamepadButton(driverGamepad, GamepadKeys.Button.TRIANGLE); // aka Y
        Supplier<Float> fwdSupplier = () -> -gamepad1.left_stick_y;
        Supplier<Float> strafeSupplier = () -> -gamepad1.left_stick_x;
        Supplier<Float> turnSupplier = () -> -gamepad1.right_stick_x;
        Command goFieldOriented = pps.cmdGoFieldOriented(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command reorient = pps.cmdSetFieldForwardDirection();

        pps.cmdSetFieldForwardDirection(0).schedule();
        pps.setDefaultCommand(goFieldOriented);

        reorientButton.whenPressed(reorient);
    }
}
