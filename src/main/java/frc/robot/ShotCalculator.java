// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Notifier;

public class ShotCalculator {
    public record ShotParameters(Rotation2d heading, Rotation2d hoodAngle, double rps) {}

    private static InterpolatingDoubleTreeMap hoodAngleLUT, rpsLUT;

    private static Notifier notifier;

    private static Supplier<Pose2d> robotPoseSupplier;
    private static Supplier<Translation2d> vrobotSupplier;

    private static volatile Translation2d rtarget;
    private static volatile ShotParameters targetParams;

    public static Pose2d getTargetPose() {
        return new Pose2d(rtarget, new Rotation2d());
    }

    public static ShotParameters getTargetParams() {
        return targetParams;
    }

    private static void updateState() {
        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d rshooter = robotPose.getTranslation().plus(Constants.Shooter.translationOffset.rotateBy(robotPose.getRotation()));

        if (robotPose.getX() > 4) {
            if (robotPose.getY() > 4) rtarget = Constants.Field.feedLeftPose;
            else rtarget = Constants.Field.feedRightPose;
        }
        else rtarget = Constants.Field.hubPose;

        Translation2d dr = rtarget.minus(rshooter);
        double distance = dr.getNorm();

        ShotParameters rawParams = new ShotParameters(
            dr.getAngle(),
            Rotation2d.fromDegrees(hoodAngleLUT.get(distance)),
            rpsLUT.get(distance)
        );

        Translation2d vrproj = vrobotSupplier.get();
        Translation3d vrobot = new Translation3d(vrproj.getX(), vrproj.getY(), 0);

        double linearVel = rawParams.rps() * Constants.Shooter.rpsToLinearVel;
        double headingRad = rawParams.heading().getRadians(), hoodAngleRad = rawParams.hoodAngle().getRadians();
        
        Translation3d vball = new Translation3d(
            Math.cos(headingRad) * Math.cos(hoodAngleRad) * linearVel,
            Math.sin(headingRad) * Math.cos(hoodAngleRad) * linearVel,
            Math.sin(hoodAngleRad) * linearVel
        );

        Translation3d vshot = vball.minus(vrobot);
        Translation2d vsproj = new Translation2d(vshot.getX(), vshot.getY());

        targetParams = new ShotParameters(
            vsproj.getAngle(),
            Rotation2d.fromRadians(Math.atan2(vshot.getZ(), vsproj.getNorm())),
            vshot.getNorm() / Constants.Shooter.rpsToLinearVel
        );
    }

    public static void configure(Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d> vrobotSupplier) {
        ShotCalculator.robotPoseSupplier = robotPoseSupplier;
        ShotCalculator.vrobotSupplier = vrobotSupplier;

        hoodAngleLUT = new InterpolatingDoubleTreeMap();
        hoodAngleLUT.put(1.4, 22.0);
        hoodAngleLUT.put(4.4, 37.0);

        rpsLUT = new InterpolatingDoubleTreeMap();
        rpsLUT.put(1.4, 40.0);
        rpsLUT.put(4.4, 50.0);

        notifier = new Notifier(ShotCalculator::updateState);
        notifier.startPeriodic(Constants.odomPeriod);
    }
}
