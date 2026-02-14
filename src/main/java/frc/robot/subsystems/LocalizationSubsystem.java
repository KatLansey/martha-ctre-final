// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelights.LimelightHelpers;
import frc.robot.Limelights.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalizationSubsystem extends SubsystemBase {
  /** Creates a new LocalizationSubsystem. */
  private Field2d field;
  private LimelightSubsystem limelight42;
  private LimelightSubsystem limelight4;
  private LimelightSubsystem limelight3;

  public LocalizationSubsystem(Field2d field, LimelightSubsystem limelight42, LimelightSubsystem limelight4, LimelightSubsystem limelight3) {
    this.field = field;
    this.limelight42 = limelight42;
    this.limelight4 = limelight4;
    this.limelight3 = limelight3;
  }

  private double x;
  private double y;

  public void updatePoseEstimation() {
    if (field != null) {
      field.setRobotPose(LimelightHelpers.getBotPose2d_wpiBlue(limelight42.getLimelightName()));
      SmartDashboard.putNumber("RobotX", field.getRobotPose().getX());
      SmartDashboard.putNumber("RobotY", field.getRobotPose().getY());
      x = field.getRobotPose().getX();
      y = field.getRobotPose().getY();
      SmartDashboard.putNumber("DistanceFromHub", Math.sqrt(Math.pow(Math.abs(x-4), 2)+Math.pow(Math.abs(y-5), 2)));
    }
    
  }

  @Override
  public void periodic() {
    updatePoseEstimation();
    // This method will be called once per scheduler run
  }
}
