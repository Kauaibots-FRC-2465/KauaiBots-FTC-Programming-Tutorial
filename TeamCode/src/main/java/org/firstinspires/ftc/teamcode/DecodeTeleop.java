package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroPathingSubsystem;

import java.util.function.Supplier;

@TeleOp(name = "2025 Decode TeleOp", group = "20311")
public class DecodeTeleop extends CommandOpMode {
    private PedroPathingSubsystem pps;
    private FlywheelSubsystem fs;
    private VoltageSensor controlHubVSensor = null;
    private GamepadEx driverGamepad, engineerGamepad;
    private GamepadButton reorientButton, driverCentricButton, unjamButton;

    @Override
    public void initialize() {
        try {
            controlHubVSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        }
        catch (Exception ignored) {
            throw new IllegalArgumentException("Failed to get Control Hub voltage sensor."+
                    "  You may need to power cycle the Robot.");
        }
        pps = new PedroPathingSubsystem(hardwareMap, new Pose(0+17.2/2, 24, 0));
        fs = new FlywheelSubsystem(hardwareMap, controlHubVSensor, 28, 3);
        fs.addFlywheelMotor("shooter1", DcMotorSimple.Direction.REVERSE);
        fs.addFlywheelMotor("shooter2", DcMotorSimple.Direction.REVERSE);
        driverGamepad = new GamepadEx(gamepad1);
        engineerGamepad = new GamepadEx(gamepad2);
        reorientButton = new GamepadButton(driverGamepad, GamepadKeys.Button.TRIANGLE); // aka Y
        driverCentricButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CIRCLE); // aka B
        unjamButton = new GamepadButton(driverGamepad, GamepadKeys.Button.PS); // AKA GUIDE (big green X)
        Supplier<Float> fwdSupplier = () -> -gamepad1.left_stick_y;
        Supplier<Float> strafeSupplier = () -> -gamepad1.left_stick_x;
        Supplier<Float> turnSupplier = () -> -gamepad1.right_stick_x;
        Command goFieldOriented = pps.cmdGoFieldOriented(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command goDriverCentric = pps.cmdGoDriverCentric(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command reorient = pps.cmdSetFieldForwardDirection();
        Command driverPlaysRed = pps.cmdSetDriverPose(new Pose(  0-12, 24));
        Command driverPlaysBlue = pps.cmdSetDriverPose(new Pose(144+12, 24));
        Command unjam=fs.cmdUnjam(.4, .15);

        pps.cmdSetFieldForwardDirection(0).schedule();
        pps.setDefaultCommand(goFieldOriented);
        driverPlaysRed.schedule();

        reorientButton.whenPressed(reorient).whenPressed(goFieldOriented);
        driverCentricButton.whenPressed(goDriverCentric);

        unjamButton.whileHeld(unjam);
    }
}