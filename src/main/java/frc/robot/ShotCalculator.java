// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;

public class ShotCalculator {
    private static Notifier notifier;

    private static Supplier<Pose2d> poseSupplier;
    private static Supplier<ChassisSpeeds> speedsSupplier;

    private static volatile Translation2d targetPose;
    private static volatile double targetHeading, targetHoodAngle, targetRPM;

    private static void updateState() {
        Translation2d robotPose = poseSupplier.get().getTranslation();

        if (robotPose.getX() > 4) {
            if (robotPose.getY() > 4) targetPose = Constants.Field.feedLeftPose;
            else targetPose = Constants.Field.feedRightPose;
        }
        else targetPose = Constants.Field.hubPose;

        double rawHeading = Math.atan2(
            targetPose.getY() - robotPose.getY(),
            targetPose.getX() - robotPose.getX()
        );

        targetHeading = rawHeading;
        targetHoodAngle = 0;
        targetRPM = 0;
    }

    public static Pose2d getTargetPose() {
        return new Pose2d(targetPose, new Rotation2d());
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
        ShotCalculator.poseSupplier = poseSupplier;
        ShotCalculator.speedsSupplier = speedsSupplier;
        
        notifier = new Notifier(ShotCalculator::updateState);
        notifier.startPeriodic(1 / Constants.odomFrequency);
    }
}
