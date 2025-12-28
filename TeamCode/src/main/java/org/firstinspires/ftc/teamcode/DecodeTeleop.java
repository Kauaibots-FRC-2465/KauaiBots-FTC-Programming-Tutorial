package org.firstinspires.ftc.teamcode;

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
    private GamepadEx driverGamepad;
    private GamepadButton reorientButton, driverCentricButton, testButton, testButton2, increaseP, decreaseP;
    private VoltageSensor controlHubVSensor = null;
    private VoltageSensor expansionHubVSensor = null;

    @Override
    public void initialize() {
        pps = new PedroPathingSubsystem(hardwareMap, new Pose(0+17.2/2, 24, 0));
        driverGamepad = new GamepadEx(gamepad1);
        reorientButton = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        driverCentricButton = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        testButton = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        testButton2 = new GamepadButton(driverGamepad, GamepadKeys.Button.X);
        increaseP = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        decreaseP = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        Supplier<Float> fwdSupplier = () -> -gamepad1.left_stick_y;
        Supplier<Float> strafeSupplier = () -> -gamepad1.left_stick_x;
        Supplier<Float> turnSupplier = () -> -gamepad1.right_stick_x;
        Command goFieldOriented = pps.cmdGoFieldOriented(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command goDriverCentric = pps.cmdGoDriverCentric(fwdSupplier, strafeSupplier, turnSupplier, true);
        Command reorient = pps.cmdSetFieldForwardDirection();
        Command driverPlaysRed = pps.cmdSetDriverPose(new Pose(  0-12, 24));
        Command driverPlaysBlue = pps.cmdSetDriverPose(new Pose(144+12, 24));
        String hubName = "none";
        try {
            hubName = "Control Hub";
            controlHubVSensor = hardwareMap.get(VoltageSensor.class, hubName);
            hubName = "Expansion Hub 2";
            expansionHubVSensor = hardwareMap.get(VoltageSensor.class, hubName);
        }
        catch (Exception ignored) {
            throw new IllegalArgumentException("Failed to get " + hubName + " voltage"+
                    " sensor.  You may need to power cycle the Robot.  (Did you verify the name"+
                    " you gave matches what you set in the robot configuration?)");
        }

        pps.cmdSetFieldForwardDirection(0).schedule();
        pps.setDefaultCommand(goFieldOriented);
        driverPlaysRed.schedule();

        reorientButton.whenPressed(reorient).whenPressed(goFieldOriented);
        driverCentricButton.whenPressed(goDriverCentric);

        fs = new FlywheelSubsystem(hardwareMap, controlHubVSensor, 28, 3);
        fs.addFlywheelMotor("shooter1", false);
        fs.addFlywheelMotor("shooter2", false);

        testButton.whenPressed(fs.cmdTuneKs());
        testButton2.whileHeld(fs.cmdTuneWithTelemetry(()->1500d, ()->false));
        increaseP.whenPressed(fs.cmdIncreaseP(.001d));
        decreaseP.whenPressed(fs.cmdDecreaseP(.001d));
    }
}
