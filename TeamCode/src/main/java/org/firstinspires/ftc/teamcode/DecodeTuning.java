package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

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
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GenericMotorSubsystem;

@TeleOp(name = "2025 Decode Tuning", group = "20311")
public class DecodeTuning extends CommandOpMode {
    private FlywheelSubsystem fs;
    private GenericMotorSubsystem intake;
    private VoltageSensor controlHubVSensor = null;
    private GamepadEx driverGamepad;
    private GamepadButton findMotorConstantsButton;
    private GamepadButton stopButton;
    private GamepadButton flywheelTuneButton;
    private GamepadButton dpadUpButton;
    private GamepadButton dpadDownButton;
    private GamepadButton dpadRightButton;
    private GamepadButton dpadLeftButton;
    private GamepadButton testLaunchDetectionButton;
    private GamepadButton intakeFindMotorConstantsButton;
    private GamepadButton intakeTunePositionP;
    private GamepadButton lastTuningButton=null;
    private double intakeTestRPM=500;
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
        intake = new GenericMotorSubsystem(hardwareMap, "intake", FORWARD, Motor.GoBILDA.RPM_1150.getCPR());
        driverGamepad = new GamepadEx(gamepad1);
        findMotorConstantsButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CROSS); // aka A;
        stopButton = new GamepadButton(driverGamepad, GamepadKeys.Button.CIRCLE); // aka B
        flywheelTuneButton = new GamepadButton(driverGamepad, GamepadKeys.Button.SQUARE); // aka X
        dpadUpButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        dpadDownButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        dpadRightButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT);
        dpadLeftButton = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT);
        testLaunchDetectionButton = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        intakeTunePositionP =  new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        findMotorConstantsButton.whenPressed(fs.cmdFindMotorConstants());
        stopButton.whenPressed(fs.cmdStop());
        flywheelTuneButton.whenHeld(new InstantCommand(()->lastTuningButton = flywheelTuneButton).andThen(fs.cmdTuneWithTelemetry(1500d)));
        intakeFindMotorConstantsButton = new GamepadButton(driverGamepad, GamepadKeys.Button.TRIANGLE); // aka Y;
        dpadUpButton.whenPressed(cmdDpadUpButton());
        dpadDownButton.whenPressed(cmdDpadDownButton());
        dpadRightButton.whenPressed(cmdDpadRightButton());
        dpadLeftButton.whenPressed(cmdDpadLeftButton());
        Command launch = fs.cmdSetRPM(()->1500, ()->false);
        Command detectLaunch =
            new WaitCommand(50).andThen(
            fs.cmdWaitUntilStable(),
            cmdLog("Stable "+System.nanoTime()),
            fs.cmdWaitLaunchStart(1500, 200),
            cmdLog("Launch started at "+System.nanoTime()),
            fs.cmdWaitLaunchEnd(5),
            cmdLog("Launch ended at "+System.nanoTime()));
        testLaunchDetectionButton.whenPressed(launch.raceWith(detectLaunch));
        testLaunchDetectionButton.whenPressed(
                fs.cmdSetRPM(()->1500, ()->false)
                    .raceWith(
                new WaitCommand(50).andThen(
                            fs.cmdWaitUntilStable(),
                            cmdLog("Stable "+System.nanoTime()),
                            fs.cmdWaitLaunchStart(1500, 200),
                            cmdLog("Launch started at "+System.nanoTime()),
                            fs.cmdWaitLaunchEnd(5),
                            cmdLog("Launch ended at "+System.nanoTime()))));
        intakeFindMotorConstantsButton.whenPressed(
                new InstantCommand(()->lastTuningButton=intakeFindMotorConstantsButton).andThen(
                    intake.cmdFindMotorConstants(()->intakeTestRPM)
                ));
        intakeTunePositionP.whenPressed(
                new InstantCommand(()->lastTuningButton=intakeTunePositionP).andThen(
                        intake.cmdTunePositionP()
                ));
    }

    public Command cmdDpadUpButton() {
        return new InstantCommand( ()-> {
            if (lastTuningButton == flywheelTuneButton) fs.cmdIncreaseP().schedule();
            if (lastTuningButton == intakeFindMotorConstantsButton) intake.cmdIncreaseVelocityP().schedule();
            if (lastTuningButton == intakeTunePositionP) intake.cmdIncreasePositionP().schedule();
        });
    }

    public Command cmdDpadDownButton() {
        return new InstantCommand( ()-> {
            if (lastTuningButton == flywheelTuneButton) fs.cmdDecreaseP().schedule();
            if (lastTuningButton == intakeFindMotorConstantsButton) intake.cmdDecreaseVelocityP().schedule();
            if (lastTuningButton == intakeTunePositionP) intake.cmdDecreasePositionP().schedule();
        });
    }

    public Command cmdDpadRightButton() {
        return new InstantCommand( ()-> {
            if (lastTuningButton == intakeFindMotorConstantsButton) intakeTestRPM+=100;
            if (lastTuningButton == intakeTunePositionP) intake.cmdIncreasePositionPower().schedule();
        });
    }

    public Command cmdDpadLeftButton() {
        return new InstantCommand( ()-> {
            if (lastTuningButton == intakeFindMotorConstantsButton) intakeTestRPM-=100;
            if (lastTuningButton == intakeTunePositionP) intake.cmdDecreasePositionPower().schedule();
        });
    }

    public Command cmdLog (String info) {
        return new InstantCommand(()-> Log.i("FTC20311", info));
    }
}
