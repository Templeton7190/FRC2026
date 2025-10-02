// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDPattern extends Command {
  private final DriveSubsystem m_DriveSubsystem;
  private final RobotContainer m_RobotContainer;
  private final double m_pattern;
  /** Creates a new LEDPattern. */
  public LEDPattern(DriveSubsystem driveSubsystem, RobotContainer robotContainer, double pattern) {
    this.m_DriveSubsystem = driveSubsystem;
    this.m_RobotContainer = robotContainer;
    this.m_pattern = pattern;
    addRequirements(driveSubsystem); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveSubsystem.setPattern(m_pattern);
    if (m_RobotContainer.patternNum >= LEDConstants.maxPatterns){
      m_RobotContainer.patternNum = 1;
    } else {
      m_RobotContainer.patternNum += 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
