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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalizationSubsystem extends SubsystemBase {
  /** Creates a new LocalizationSubsystem. */
  private Field2d field;
  //x, y, z, roll, pitch, yaw
  private double[] arrayPose3d;
  //x, y, 0, 0, 0, yaw
  private double[] arrayPose2d;
  private LimelightSubsystem limelight42;
  private LimelightSubsystem limelight4;
  private LimelightSubsystem limelight3;
  private LimelightSubsystem[] limelights;
  private CommandSwerveDrivetrain drivetrain;
  private Pose2d localPose = new Pose2d();
  private Pose3d localPose3d = new Pose3d();
  
  private boolean threeD = false;
  private boolean fieldUse = false;

  public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain, Field2d field, LimelightSubsystem limelight42, LimelightSubsystem limelight4, LimelightSubsystem limelight3) {
    this.drivetrain = drivetrain;
    this.field = field;
    this.limelight42 = limelight42;
    this.limelight4 = limelight4;
    this.limelight3 = limelight3;
    limelights = new LimelightSubsystem[]{limelight42, limelight4, limelight3};
    this.threeD = false;
    this.fieldUse = true;
  }

  public LocalizationSubsystem(CommandSwerveDrivetrain drivetrain, boolean threeD, LimelightSubsystem limelight42, LimelightSubsystem limelight4, LimelightSubsystem limelight3) {
    this.drivetrain = drivetrain;
    this.limelight42 = limelight42;
    this.limelight4 = limelight4;
    this.limelight3 = limelight3;
    limelights = new LimelightSubsystem[]{limelight42, limelight4, limelight3};
    this.threeD = threeD;
    if (threeD) {
      this.localPose3d = new Pose3d(
        drivetrain.getState().Pose.getX(), 
        drivetrain.getState().Pose.getY(),
        0,
        drivetrain.getPigeon2().getRotation3d());
    }
  }

  private double x;
  private double y;
  private double z;
  private double rotation;
  private double hubX = 4.06; //159.75 inches
  private double hubY = 4;
  private double hubZ = 0;

  public Pose2d getPose() {
    return localPose;
  }

  public Pose3d getPose3d() {
    return localPose3d;
  }

  public void updateEstimator(boolean limelight3dpose) {
    boolean limelightEstimator = false;
    for (LimelightSubsystem limelightSelect : limelights) {
      if (limelightSelect.hasTarget()) {
        SmartDashboard.putString("Estimator", limelightSelect.getLimelightName());
        if (limelight3dpose) {
          localPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelightSelect.getLimelightName());
        } else {
          localPose = LimelightHelpers.getBotPose2d_wpiBlue(limelightSelect.getLimelightName());
        }
        limelightEstimator = true;
        return;
      }
    }
    if (!limelightEstimator) {
      if (!SmartDashboard.getString("Estimator", "N/A").equals("Drivetrain")) {
        drivetrain.resetPose(localPose);
      }
      SmartDashboard.putString("Estimator", "Drivetrain");
      if (limelight3dpose) {
        localPose3d = new Pose3d(
          drivetrain.getState().Pose.getX(), 
          drivetrain.getState().Pose.getY(),
          localPose3d.getZ(),
          drivetrain.getPigeon2().getRotation3d());
      } else {
        localPose = drivetrain.getState().Pose;
      }
    }
  }

  public void postPoseEstimation() {
    if (!threeD) {
      if (!fieldUse) {
        arrayPose2d = LimelightHelpers.pose2dToArray(localPose);
        
        x = arrayPose2d[0];
        y = arrayPose2d[1];

        rotation = arrayPose2d[5] = drivetrain.getRobotAngle();

        SmartDashboard.putNumberArray("RobotPose2d", arrayPose2d);

      } else {
        field.setRobotPose(localPose);

        x = field.getRobotPose().getX();
        y = field.getRobotPose().getY();
        rotation = field.getRobotPose().getRotation().getDegrees();

        //field.setRobotPose(x, y, new Rotation2d((drivetrain.getRobotAngle()%365)*(Math.PI/180)));
      }

      SmartDashboard.putNumber("RobotX", x);
      SmartDashboard.putNumber("RobotY", y);
      SmartDashboard.putNumber("Rotation", rotation);
      // √((x-hubX)^2 + (y-hubY)^2) = distance
      SmartDashboard.putNumber("DistanceFromHub", Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2)));
    } else {
      arrayPose3d = LimelightHelpers.pose3dToArray(localPose3d);

      x = arrayPose3d[0];
      y = arrayPose3d[1];
      z = arrayPose3d[2];
      
      SmartDashboard.putNumberArray("RobotPose3d", arrayPose3d);

      SmartDashboard.putNumber("RobotX", x);
      SmartDashboard.putNumber("RobotY", y);
      SmartDashboard.putNumber("RobotZ", z);

      
      SmartDashboard.putNumber("Rotation", rotation);

      // √((x-hubX)^2 + (y-hubY)^2) = distance
      SmartDashboard.putNumber("DistanceFromHub", Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2)));
      SmartDashboard.putNumber("DistanceFromHub3D", Math.sqrt(Math.pow(Math.abs(x-hubX), 2)+Math.pow(Math.abs(y-hubY), 2)+Math.pow(Math.abs(z-hubZ), 2)));
    }
  }

  public double calcHubAngle() {
    return Math.atan(Math.abs(y-hubY)/Math.abs(x-hubX))*(180/Math.PI);
  }

  @Override
  public void periodic() {
    updateEstimator(false);
    postPoseEstimation();
    // This method will be called once per scheduler run
  }
}
