package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

public class PedroPathingSubsystem extends SubsystemBase {
    private final Follower follower;
    private float fieldForwardRadians = 0;  // The field heading considered to be "forward"
    private boolean teleopActive = false, teleopBraking = true; // Did we already send startTeleopDrive? Are we in braking mode?
    private Pose driverPose = new Pose(0, 0);

    public PedroPathingSubsystem(final HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public PedroPathingSubsystem(final HardwareMap hardwareMap, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void periodic() {
        follower.update();
        try {
            Drawing.drawDebug(follower);
        } catch (Exception e) {
            Log.e("20311", "Attempting to draw the robot position in Panels dashboard failed.", e);
        }
    }

    /**
     * Applies forward/strafe/rotate power with a heading offset to the robot's drive
     *
     * @param forward          The power applied in the direction of {@code headingOffset} [Range: -1.0 to 1.0].
     * @param strafe           The power applied 90° CCW of {@code headingOffset} [Range: -1.0 to 1.0].
     * @param turn             The power applied to rotate the robot CW/CCW [Range: -1.0 to 1.0].
     * @param headingOffset    The direction, in radians, that defines "forward"
     * @param withBraking      Whether to enable ZeroPowerBehavior.BRAKE (true) or FLOAT (false).
     */
    private void subDriveRobot(float forward, float strafe, float turn, float headingOffset, boolean withBraking) {
        if (!teleopActive || teleopBraking != withBraking) follower.startTeleopDrive(withBraking);
        teleopBraking = withBraking;
        teleopActive = true;
        follower.setTeleOpDrive(forward, strafe, turn, false, headingOffset);
    }

    /** Create command to set the field-oriented "forward" direction to a fixed value.
     * @param fieldForwardRadians The new forward heading in radians.
     * @return An {@link InstantCommand}
     */
    public Command cmdSetFieldForwardDirection(float fieldForwardRadians) {
        return new InstantCommand( ()-> this.fieldForwardRadians=fieldForwardRadians );
    }

    /** Create command to set the field-oriented "forward" direction to the robot's heading, taken at the time the command is scheduled
     * @return An {@link InstantCommand}
     */
    public Command cmdSetFieldForwardDirection() {
        return new InstantCommand( ()-> this.fieldForwardRadians=(float)follower.getHeading());
    }

    /** Create command to initiate field-oriented driving.
     * @param forward     Supplier for the backward/forward driving power [-1.0, 1.0].
     * @param strafe      Supplier for the right/left strafing power [-1.0, 1.0].
     * @param turn        Supplier for the cw/ccw rotational power [-1.0, 1.0].
     * @param withBraking If true, the robot will use active braking when inputs are neutral.
     * @return An interruptible {@link RunCommand}
     * @see #cmdSetFieldForwardDirection(float)
     * @see #cmdSetFieldForwardDirection()
     */
    public Command cmdGoFieldOriented(Supplier<Float> forward, Supplier<Float> strafe,
                                        Supplier<Float> turn, boolean withBraking) {
        return new RunCommand(
                () -> this.subDriveRobot(forward.get(), strafe.get(), turn.get(), fieldForwardRadians, withBraking),
                this // Require this subsystem to run this command
        );
    }

    /** Create command to set the location where the driver stands while driving the robot.
     * @return An {@link InstantCommand}
     */
    public Command cmdSetDriverPose(Pose driverPose) {
        return new InstantCommand( ()-> this.driverPose=driverPose);
    }

    /** Create command to initiate driver-centric driving.
     * @param forward     Supplier for the backward/forward driving power [-1.0, 1.0].
     * @param strafe      Supplier for the right/left strafing power [-1.0, 1.0].
     * @param turn        Supplier for the cw/ccw rotational power [-1.0, 1.0].
     * @param withBraking If true, the robot will use active braking when inputs are neutral.
     * @return An interruptible {@link RunCommand}
     */
    public Command cmdGoDriverCentric(Supplier<Float> forward, Supplier<Float> strafe,
                                      Supplier<Float> turn, boolean withBraking) {
        return new RunCommand(
                () -> {
                    float driverForward = (float) Math.atan2(
                            follower.getPose().getY()- driverPose.getY(),
                            follower.getPose().getX()- driverPose.getX()
                    );
                    this.subDriveRobot(forward.get(), strafe.get(), turn.get(), driverForward, withBraking);
                },
                this // Require this subsystem to run this command
        );
    }
}
