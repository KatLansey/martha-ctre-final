// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Limelights.newLimelightCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Limelights.LimelightSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimToPass extends SequentialCommandGroup {
  /** Creates a new AimToPass. */
  private LimelightSubsystem limelight;
  public AimToPass(CommandSwerveDrivetrain drivetrain, CommandXboxController xboxController, LimelightSubsystem limelight, int pipeline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.limelight = limelight;
    addCommands(
      new InstantCommand(() -> limelight.setPipeline(pipeline)),
      new AimAtAngle(drivetrain, xboxController, limelight.getPassingRotation(getOffsetApriltag()))
    );
  }

  public double getOffsetApriltag() { //do red later
    switch (limelight.getID()) { //measurements needed
      case(17): // blue left trench
        return 0; 
      case(18): // blue left side hub
        return 0;
      case(19): // blue mid side left hub
        return 0;
      case(20): // blue mid side mid hub
        return 0;
      case(21): // blue right side hub
        return 0;
      case(22): // blue right trench
        return 0;
      default:
        return 0;
    }
  }
}
