package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.65); // Added per https://pedropathing.com/docs/pathing/tuning/setup "Setting your robot's mass"

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0d)
            .rightFrontMotorName("rightFront") // The name we gave to the right front mecanum motor in the Robot Configuration
            .leftFrontMotorName("leftFront") // The name we gave to the left front mecanum motor in the Robot Configuration
            .rightRearMotorName("rightBack")  // The name we gave to the right rear mecanum motor in the Robot Configuration
            .leftRearMotorName("leftBack")  // The name we gave to the left rear mecanum motor in the Robot Configuration
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(130.0d) // The forward/backward (center of left rail) pod is 130mm left of center
            .strafePodX(213.0d) // The left/right pod (front left rail) is 213mm forward of center
            .distanceUnit(DistanceUnit.MM) // We decided to measure in mm rather than inches
            .hardwareMapName("odo1") // The name we gave to the odometry computer in the Robot Configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants) // Added per https://pedropathing.com/docs/pathing/tuning/setup "Adding drivetrain constants" - "Mecanum"
                .build();
    }
}