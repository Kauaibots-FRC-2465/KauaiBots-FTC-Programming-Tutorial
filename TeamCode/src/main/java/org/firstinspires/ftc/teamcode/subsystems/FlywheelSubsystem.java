package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FlywheelSubsystem extends SubsystemBase {
    HardwareMap hardwareMap;
    MotorEx[] flywheelMotors;
    double countsPerOutputRotation;
    double flywheelDiameter;

    public FlywheelSubsystem (HardwareMap hardwareMap, MotorEx[] flywheelMotors, double countsPerOutputRotation, double flywheelDiameter) {
        this.hardwareMap = hardwareMap;
        this.flywheelMotors = flywheelMotors;
        this.countsPerOutputRotation = countsPerOutputRotation;
        this.flywheelDiameter = flywheelDiameter;


        for (MotorEx flywheelMotor: flywheelMotors) {
            flywheelMotor.getCPR();
        }
        MotorEx test = new MotorEx(hardwareMap,"test", Motor.GoBILDA.BARE);
        MotorGroup testgroup = new MotorGroup(test);
        test.setVelocity(1, AngleUnit.DEGREES);
        test.set(10);
    }


}
