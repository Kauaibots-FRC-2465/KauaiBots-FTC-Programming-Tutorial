package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PedroPathingSubsystem;

@TeleOp(name = "2025 Decode Teleop", group = "20311")
public class DecodeTeleop extends CommandOpMode {
    PedroPathingSubsystem pps;
        private GamepadEx driverGamepad;
        private GamepadButton reorientButton ;

    @Override
    public void initialize() {
        pps = new PedroPathingSubsystem(hardwareMap);
        driverGamepad = new GamepadEx(gamepad1);
        reorientButton = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        pps.cmdSetFieldForwardDirection(0).schedule();

        pps.setDefaultCommand(
                pps.cmdDriveFieldOriented(
                        () -> -gamepad1.left_stick_y,
                        () -> -gamepad1.left_stick_x,
                        () -> -gamepad1.right_stick_x,
                        true
                )
        );
        reorientButton.whenPressed(pps.cmdSetFieldForwardDirection());
    }
}
