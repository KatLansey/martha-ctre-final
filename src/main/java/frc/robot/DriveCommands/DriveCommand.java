// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.DriveCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new Drive. */
  CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double ySpeed;
  double xSpeed;
  double zSpeed;

  public DriveCommand(CommandSwerveDrivetrain drivetrain, double xSpeed, double ySpeed, double zSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zSpeed = zSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(drive.withVelocityX(CommandConstants.MaxSpeed/3*-ySpeed).withVelocityY(CommandConstants.MaxSpeed/3*xSpeed).withRotationalRate(CommandConstants.MaxAngularRate*zSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
