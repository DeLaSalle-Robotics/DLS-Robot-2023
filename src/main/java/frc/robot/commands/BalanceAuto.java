// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAuto extends SequentialCommandGroup {
  /** Creates a new BalanceAuto. */
  DrivetrainSubsystem m_drivetrain;
  public BalanceAuto(DrivetrainSubsystem _drive) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SimpleAuto(_drive),
      new ShortDrive(_drive),
      new Balance(_drive)

    );
  }
}
