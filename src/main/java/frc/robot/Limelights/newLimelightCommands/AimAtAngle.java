// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Limelights.newLimelightCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.xml.sax.SAXException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.CommandConstants;
import frc.robot.Limelights.LimelightSubsystem;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class AimAtAngle extends Command {
  private frc.robot.subsystems.CommandSwerveDrivetrain drivetrain;

  private PIDController thetaController = new PIDController(1, 0, 0.1);

  private boolean targeting = false;
  private CommandXboxController controller;
  private double angle = 0;
  private double setpoint = 0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(CommandConstants.MaxSpeed * 0.1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public AimAtAngle(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, double angle) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.angle = angle;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Aligning", true);
    SmartDashboard.putNumber("Align Goal", angle);
    SmartDashboard.putNumber("targeting error", drivetrain.getRobotAngle() - angle);

    double thetaOutput = 0;

    double xOutput = controller.getLeftY();
    double yOutput = controller.getLeftX();

		thetaController.setSetpoint(setpoint);

    if (!thetaController.atSetpoint()){
      thetaOutput = thetaController.calculate(drivetrain.getRobotAngle(), angle);
    }
    
    drivetrain.setControl(
      drive.withVelocityX(-xOutput*CommandConstants.MaxSpeed)
          .withVelocityY(-yOutput*CommandConstants.MaxSpeed)
          .withRotationalRate(thetaOutput*CommandConstants.MaxAngularRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Aligning", false);
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}