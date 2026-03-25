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
    private static Notifier notifier;

    private static Supplier<Translation2d> xrobotSupplier;
    private static Supplier<Translation2d> vrobotSupplier;

    private static volatile Translation2d xtarget;
    private static volatile double targetHeading, targetHoodAngle, targetPercent;

    private static void updateState() {
        Translation2d xrobot = xrobotSupplier.get();

        if (xrobot.getX() > 4) {
            if (xrobot.getY() > 4) xtarget = Constants.Field.feedLeftPose;
            else xtarget = Constants.Field.feedRightPose;
        }
        else xtarget = Constants.Field.hubPose;

        targetHeading = xtarget.minus(xrobot).getAngle().getRadians();
        targetHoodAngle = 0;
        targetPercent = 0.5;
    }

    public static Pose2d getTargetPose() {
        return new Pose2d(xtarget, new Rotation2d());
    }

    public static double getTargetHeading() {
        return targetHeading;
    }

    public static double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public static double getTargetPercent() {
        return targetPercent;
    }

    public static void configure(Supplier<Translation2d> xrobotSupplier, Supplier<Translation2d> vrobotSupplier) {
        ShotCalculator.xrobotSupplier = xrobotSupplier;
        ShotCalculator.vrobotSupplier = vrobotSupplier;
        
        notifier = new Notifier(ShotCalculator::updateState);
        notifier.startPeriodic(Constants.odomPeriod);
    }
}
