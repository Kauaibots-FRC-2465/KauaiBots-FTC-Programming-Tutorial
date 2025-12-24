package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class PedroPathingSubsystem extends SubsystemBase {
    private final Follower follower;
    public PedroPathingSubsystem(final HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    private float fieldForwardRadians=0;  // The field heading considered to be "forward"
    private boolean teleopActive = false, teleopBraking = true; // Did we already send startTeleopDrive? Are we in braking mode?

    @Override
    public void periodic() {
        follower.update();
        try {
            Drawing.drawDebug(follower);
            Drawing.sendPacket();
        } catch (Exception e) {
            Log.e("20311", "Attempting to draw the robot position in FTC Dashboard failed." + e);
        }
    }

    /**
     * applies foward/strafe/rotate power with a rotation offset to the robot's drive
     *
     * @param forward          The power applied in the {@code forwardDirection} [Range: -1.0 to 1.0].
     * @param strafe           The power applied 90° CCW from the {@code forwardDirection} [Range: -1.0 to 1.0].
     * @param headingOffset    The rotational power (CCW is positive) [Range: -1.0 to 1.0].
     * @param withBraking      Whether to enable ZeroPowerBehavior.BRAKE (true) or FLOAT (false).
     */
    private void subDriveRobot(float forward, float strafe, float turn, float headingOffset, boolean withBraking) {
        if (!teleopActive || teleopBraking != withBraking) follower.startTeleopDrive(withBraking);
        teleopBraking = withBraking;
        teleopActive = true;
        follower.setTeleOpDrive(forward, strafe, turn, false, headingOffset);
    }

    /** Create command to set the field-centric "forward" direction to a fixed value.
     * @param fieldForwardRadians The new forward heading in radians.
     * @return An {@link InstantCommand}
     */
    public Command cmdSetFieldForwardDirection(float fieldForwardRadians) {
        return new InstantCommand( ()-> this.fieldForwardRadians=fieldForwardRadians );
    }

    /** Create command to set the field-centric "forward" direction to the robots heading, taken at the time the command is scheduled
     * @return An {@link InstantCommand}
     */
    public Command cmdSetFieldForwardDirection() {
        return new InstantCommand( ()-> this.fieldForwardRadians=(float)follower.getHeading());
    }

    /** Create command to initiate field centric driving.
     * @param forward     Supplier for the backward/forward driving power [-1.0, 1.0].
     * @param strafe      Supplier for the right/left strafing power [-1.0, 1.0].
     * @param turn        Supplier for the cw/ccw rotational power [-1.0, 1.0].
     * @param withBraking If true, the robot will use active braking when inputs are neutral.
     * @return An interruptable {@link RunCommand}
     * * @see #setFieldForwardDriection(float)
     * * @see #setFieldForwardDriection()
     */
    public Command cmdDriveFieldCentric(Supplier<Float> forward, Supplier<Float> strafe,
                                        Supplier<Float> turn, boolean withBraking) {
        return new RunCommand(
                () -> this.subDriveRobot(forward.get(), strafe.get(), turn.get(), fieldForwardRadians, withBraking),
                this // Require this subsystem to run this command
        );
    }
}
