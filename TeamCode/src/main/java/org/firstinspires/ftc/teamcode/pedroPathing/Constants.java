package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants) // Added per https://pedropathing.com/docs/pathing/tuning/setup "Adding drivetrain constants" - "Mecanum"
                .build();
    }
}
