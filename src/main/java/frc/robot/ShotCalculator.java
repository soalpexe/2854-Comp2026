// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;

public class ShotCalculator {
    public record ShotParameters(Rotation2d heading, Rotation2d hoodAngle, double rps) {}

    private static Notifier notifier;

    private static Supplier<Pose2d> robotPoseSupplier;
    private static Supplier<Translation2d> vrobotSupplier;

    private static volatile Translation2d rtarget;
    private static volatile ShotParameters targetParams;

    private static void updateState() {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d rshooter = robotPose.getTranslation().plus(Constants.Shooter.translationOffset.rotateBy(robotPose.getRotation()));

        if (robotPose.getX() > 4) {
            if (robotPose.getY() > 4) rtarget = Constants.Field.feedLeftPose;
            else rtarget = Constants.Field.feedRightPose;
        }
        else rtarget = Constants.Field.hubPose;

        ShotParameters rawParams = new ShotParameters(
            rtarget.minus(rshooter).getAngle(),
            Rotation2d.fromDegrees(30),
            50
        );

        targetParams = rawParams;
    }

    public static Pose2d getTargetPose() {
        return new Pose2d(rtarget, new Rotation2d());
    }

    public static ShotParameters getTargetParams() {
        return targetParams;
    }

    public static void configure(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> vrobotSupplier) {
        ShotCalculator.robotPoseSupplier = robotPoseSupplier;
        ShotCalculator.vrobotSupplier = vrobotSupplier;
        
        notifier = new Notifier(ShotCalculator::updateState);
        notifier.startPeriodic(Constants.odomPeriod);
    }
}
