// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
    public Vision() {}

    public Optional<PoseEstimate> getPoseEstimate(String cameraID) {
        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(cameraID);
        if (rawFiducials.length == 0) return Optional.empty();

        for (RawFiducial rawFiducial : rawFiducials) {
            if (rawFiducial.ambiguity > Constants.Vision.maxAmbiguity) return Optional.empty();
        }

        PoseEstimate estimate = Utilities.getAlliance() == Alliance.Red ?
            LimelightHelpers.getBotPoseEstimate_wpiRed(cameraID) :
            LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraID);

        return Optional.of(estimate);
    }

    public ArrayList<PoseEstimate> getPoseEstimates() {
        String[] camIDs = {
            Constants.Vision.leftCamID,
            Constants.Vision.rightCamID
        };

        ArrayList<PoseEstimate> estimates = new ArrayList<>();

        for (String camID : camIDs) {
            Optional<PoseEstimate> estimate = getPoseEstimate(camID);
            if (estimate.isPresent()) estimates.add(estimate.get());
        }

        return estimates;
    }

    @Override
    public void periodic() {}
}
