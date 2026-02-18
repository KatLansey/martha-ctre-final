// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelights.LimelightHelpers;
import frc.robot.Limelights.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalizationSubsystem extends SubsystemBase {
  /** Creates a new LocalizationSubsystem. */
  private Field2d field;
  private LimelightSubsystem limelight42;
  private LimelightSubsystem limelight4;
  private LimelightSubsystem limelight3;
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d localPose = new Pose2d();
  private Pose3d localPose3d = new Pose3d();

  public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field, LimelightSubsystem limelight42, LimelightSubsystem limelight4, LimelightSubsystem limelight3) {
    this.drivetrain = drivetrain;
    this.field = field;
    this.limelight42 = limelight42;
    this.limelight4 = limelight4;
    this.limelight3 = limelight3;
  }

  private double x;
  private double y;
  private double hubX = 4.06; //159.75 inches
  private double hubY = 4;

  public Pose2d getEstimator() {
    if(limelight42.hasTarget()) {
      SmartDashboard.putString("Estimator", "Limelight42");
      localPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight42.getLimelightName());
    } else if (limelight4.hasTarget()) {
      SmartDashboard.putString("Estimator", "Limelight4");
      localPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight4.getLimelightName());
    } else if (limelight3.hasTarget()) {
      SmartDashboard.putString("Estimator", "Limelight3");
      localPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight3.getLimelightName());
    } else {
      if (!SmartDashboard.getString("Estimator", "N/A").equals("Drivetrain")) {
        drivetrain.resetPose(localPose);
      }
      SmartDashboard.putString("Estimator", "Drivetrain");
      localPose = drivetrain.getState().Pose;
    }
    return localPose;
  }

  public void updatePoseEstimation() {
    if (field != null) {
      field.setRobotPose(getEstimator().getMeasureX(), getEstimator().getMeasureY(), new Rotation2d(drivetrain.getRobotAngle()));

      SmartDashboard.putNumber("RobotX", field.getRobotPose().getX());
      SmartDashboard.putNumber("RobotY", field.getRobotPose().getY());
      x = field.getRobotPose().getX();
      y = field.getRobotPose().getY();
      // √((x-hubX)^2 + (y-hubY)^2) = distance
      SmartDashboard.putNumber("DistanceFromHub", Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2)));
    }
    
  }

  public double calcHubAngle() {
    return Math.atan(Math.abs(y-hubY)/Math.abs(x-hubX))*(180/Math.PI);
  }

  public Pose3d get3DEstimator() {
    if(limelight42.hasTarget()) {
      SmartDashboard.putString("Estimator3D", "Limelight42");
      localPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelight42.getLimelightName());
    } else if (limelight4.hasTarget()) {
      SmartDashboard.putString("Estimator3D", "Limelight4");
      localPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelight4.getLimelightName());
    } else if (limelight3.hasTarget()) {
      SmartDashboard.putString("Estimator3D", "Limelight3");
      localPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelight3.getLimelightName());
    } else {
      if (!SmartDashboard.getString("Estimator3D", "N/A").equals("Drivetrain")) {
        //drivetrain.resetPose(localPose3d);
      }
      SmartDashboard.putString("Estimator3D", "Drivetrain");
      //localPose3d = drivetrain.getState().Pose3d;
    }
    return localPose3d;
  }

  public void updatePose3dEstimation() {
    if (field != null) {
      field.setRobotPose(getEstimator().getMeasureX(), getEstimator().getMeasureY(), new Rotation2d(drivetrain.getRobotAngle()));

      SmartDashboard.putNumber("RobotX", field.getRobotPose().getX());
      SmartDashboard.putNumber("RobotY", field.getRobotPose().getY());
      x = field.getRobotPose().getX();
      y = field.getRobotPose().getY();
      // √((x-hubX)^2 + (y-hubY)^2) = distance
      SmartDashboard.putNumber("DistanceFromHub", Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2)));
    }
    
  }

  @Override
  public void periodic() {
    updatePoseEstimation();
    // This method will be called once per scheduler run
  }
}
