// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
    public Vision() {}

    public Pose2d getPoseEstimate(String cameraID) {
        Pose2d estimate = Utilities.getAlliance() == Alliance.Red ?
            LimelightHelpers.getBotPose2d_wpiRed(cameraID) :
            LimelightHelpers.getBotPose2d_wpiBlue(cameraID);

        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(cameraID);

        for (RawFiducial rawFiducial : rawFiducials) {
            if (rawFiducial.ambiguity > 0.7) return null;
        }

        return estimate;
    }

    public Pose2d[] getPoseEstimates() {
        return new Pose2d[] {
            getPoseEstimate(Constants.Vision.rightCamID)
        };
    }

    @Override
    public void periodic() {}
}
