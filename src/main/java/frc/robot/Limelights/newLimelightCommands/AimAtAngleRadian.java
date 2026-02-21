// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Limelights.newLimelightCommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.CommandConstants;

public class AimAtAngleRadian extends Command {
  private frc.robot.subsystems.CommandSwerveDrivetrain drivetrain;

  private PIDController thetaController = new PIDController(0.025, 0, 0);

  private boolean targeting = false;
  private CommandXboxController controller;
  private double radian = 0;
  private double tolerance = Math.toRadians(4);
  private double setpoint = 0;
  private double currentRadian = 0;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(CommandConstants.MaxSpeed * 0.1);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public AimAtAngleRadian(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, double angle) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.radian = angle*(Math.PI/180);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentRadian = Math.abs((drivetrain.getRobotAngle()*(Math.PI/180))%(2*Math.PI));
    SmartDashboard.putBoolean("Aligning", true);
    SmartDashboard.putNumber("Align Goal", radian);
    SmartDashboard.putNumber("targeting error", currentRadian - radian);
    SmartDashboard.putNumber("pid targeting error", thetaController.getError());
    SmartDashboard.putString("Robot Current Radian", Math.round(currentRadian/Math.PI*100.0)/100.0 + "pi");
    

    double thetaOutput = 0;

    double xOutput = controller.getLeftY();
    double yOutput = controller.getLeftX();

		thetaController.setSetpoint(radian);

    if (!thetaController.atSetpoint()){
      thetaOutput = thetaController.calculate(currentRadian, radian);
      SmartDashboard.putBoolean("At Setpoint", false);
    } else {
      SmartDashboard.putBoolean("At Setpoint", true);
    }
    drivetrain.setControl(
      drive.withVelocityX(-xOutput*CommandConstants.MaxSpeed)
          .withVelocityY(-yOutput*CommandConstants.MaxSpeed)
          .withRotationalRate(thetaOutput*(CommandConstants.MaxAngularRate/3)));
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
    return Math.abs(currentRadian - radian) < tolerance;
  }
}