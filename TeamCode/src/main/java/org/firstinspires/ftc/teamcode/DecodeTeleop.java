package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroPathingSubsystem;

import java.util.function.Supplier;

@TeleOp(name = "2025 Decode TeleOp", group = "20311")
public class DecodeTeleop extends CommandOpMode {
    private PedroPathingSubsystem pps;
    @Override
    public void initialize() {
        pps = new PedroPathingSubsystem(hardwareMap);
        Supplier<Float> fwdSupplier = () -> -gamepad1.left_stick_y;
        Supplier<Float> strafeSupplier = () -> -gamepad1.left_stick_x;
        Supplier<Float> turnSupplier = () -> -gamepad1.right_stick_x;
        Command goFieldOriented = pps.cmdGoFieldOriented(fwdSupplier, strafeSupplier, turnSupplier, true);

        pps.cmdSetFieldForwardDirection(0).schedule();
        pps.setDefaultCommand(goFieldOriented);
    }
}
