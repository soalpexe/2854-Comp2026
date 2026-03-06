// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShotCalculator {
    private static Notifier notifier;

    private static Supplier<Pose2d> poseSupplier;
    private static Supplier<ChassisSpeeds> speedsSupplier;

    private static volatile double targetHeading, targetHoodAngle, targetRPM;

    private static void updateState() {
        Pose2d robotPose = poseSupplier.get();
        Pose2d hubPose = Utilities.getAlliance() == Alliance.Red ? Constants.Field.redHubPose : Constants.Field.blueHubPose;

        double rawHeading = Math.atan2(
            hubPose.getY() - robotPose.getY(),
            hubPose.getX() - robotPose.getX()
        );

        targetHeading = rawHeading;
        targetHoodAngle = 0;
        targetRPM = 0;
    }

    public static double getTargetHeading() {
        return targetHeading;
    }

    public static double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public static double getTargetRPM() {
        return targetRPM;
    }

    public static void configure(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
        notifier = new Notifier(ShotCalculator::updateState);
        notifier.startPeriodic(1 / Constants.Drivetrain.odomFrequency);
        
        ShotCalculator.poseSupplier = poseSupplier;
        ShotCalculator.speedsSupplier = speedsSupplier;
    }
}
