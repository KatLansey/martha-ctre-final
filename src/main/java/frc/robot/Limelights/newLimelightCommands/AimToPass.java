// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Limelights.newLimelightCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DriveCommands.DriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimToPass extends SequentialCommandGroup {
  /** Creates a new AimToPass. */
  public AimToPass() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new DriveCommand(null, 0, 0, 0)
    );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
