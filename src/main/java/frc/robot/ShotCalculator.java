// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotCalculator {
    public static double targetHeading, targetHoodAngle, targetRPM;

    public static void updateState(Pose2d robotPose, ChassisSpeeds speeds) {
        double rawHeading = Math.atan2(
            Constants.Field.hubPose.getY() - robotPose.getY(),
            Constants.Field.hubPose.getX() - robotPose.getX()
        );

        targetHeading = rawHeading;
        targetHoodAngle = 0;
        targetRPM = 0;
    }
}
