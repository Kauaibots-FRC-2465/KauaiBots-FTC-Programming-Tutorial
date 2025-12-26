package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroPathingSubsystem;

import java.util.function.Supplier;

@TeleOp(name = "2025 Decode TeleOp", group = "20311")
public class DecodeTeleop extends CommandOpMode {
    private PedroPathingSubsystem pps;
    private GamepadEx driverGamepad;
    private GamepadButton reorientButton, driverCentricButton;

    @Override
    public void initialize() {
        pps = new PedroPathingSubsystem(hardwareMap, new Pose(0+17.2/2, 24, 0));
        driverGamepad = new GamepadEx(gamepad1);
        reorientButton = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        driverCentricButton = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        Supplier<Float> fwdSupplier = () -> -gamepad1.left_stick_y;
        Supplier<Float> strafeSupplier = () -> -gamepad1.left_stick_x;
        Supplier<Float> turnSupplier = () -> -gamepad1.right_stick_x;
        Command goFieldOriented = pps.cmdGoFieldOriented(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command goDriverCentric = pps.cmdGoDriverCentric(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command reorient=pps.cmdSetFieldForwardDirection();
        Command driverPlaysRed = pps.cmdSetDriverPose(new Pose(  0-12, 24));
        Command driverPlaysBlue = pps.cmdSetDriverPose(new Pose(144+12, 24));

        pps.cmdSetFieldForwardDirection(0).schedule();
        pps.setDefaultCommand(goFieldOriented);
        driverPlaysRed.schedule();

        reorientButton.whenPressed(reorient).whenPressed(goFieldOriented);
        driverCentricButton.whenPressed(goDriverCentric);
    }
}
