// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarSubsystem extends SubsystemBase {
  /** Creates a new LidarSubsystem. */
  private LaserCan lc;
  private LaserCan.Measurement measurement;

  public LidarSubsystem() {
    lc = new LaserCan(0);
  }

  public void checkInteruption() {
    if (measurement.distance_mm<100) {
      SmartDashboard.putBoolean("Hypothetical climber", true);
    } else { 
      SmartDashboard.putBoolean("Hypothetical climber", false);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    measurement = lc.getMeasurement();
    SmartDashboard.putNumber("Lidar measurement", measurement.distance_mm);
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      SmartDashboard.putBoolean("Lidar accurate", true);
    } else {
      SmartDashboard.putBoolean("Lidar accurate", false);
    }
  }
}
