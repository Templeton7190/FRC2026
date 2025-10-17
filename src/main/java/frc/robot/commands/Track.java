// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.tools.FileObject;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LIDARLite;
import frc.robot.subsystems.LidarSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Track extends Command {
  private final DriveSubsystem m_drive;
  private final LIDARLite m_lidarSubsystem;
  
  // target distance from tag (in meters)
  private static final double TARGET_DISTANCE_METERS = 1.0;

  // Limelight mounting geometry
  private static final double CAM_HEIGHT_METERS = 0.16;
  private static final double TAG_HEIGHT_METERS = 0.25;
  private static final double CAM_MOUNT_ANGLE_DEGREES = 0.0;

  // control gains (tune these!)
  private static final double kRot = 0.01;
  private static final double kX = 0.2;
  private static final double kY = 0.2;

  private final int id;

  public Track(DriveSubsystem drive, LIDARLite lidarSubsystem, int id) {
    this.m_drive = drive;
    this.m_lidarSubsystem = lidarSubsystem;
    this.id = id;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", id);
    switch (id) {
      case 1:
        m_drive.setPattern(0.57);
        break;
      case 2:
        m_drive.setPattern(0.81);
        break;
      case 3:
        m_drive.setPattern(0.91);
        break;
      case 4:
        m_drive.setPattern(0.61);
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("limelight")) {
      m_drive.drive(0, 0, 0, true);
      return;
    }

    // --- Limelight measurements ---
    double tx = LimelightHelpers.getTX("limelight"); // horizontal offset (deg)
    double ty = LimelightHelpers.getTY("limelight"); // vertical offset (deg)

    // --- Estimate distance using camera geometry ---
    double totalAngleDeg = CAM_MOUNT_ANGLE_DEGREES + ty;
    double totalAngleRad = Math.toRadians(totalAngleDeg);
    double distance = (TAG_HEIGHT_METERS - CAM_HEIGHT_METERS) / Math.tan(totalAngleRad);
    //double distance = m_lidarSubsystem.getDistance();
    //System.out.println(distance + " " + distanceMeters);
    if (distance <= -1) return;

    // --- Proportional control for each axis ---
    double rot = -kRot * tx; // turn to face the tag
    double xSpeed = -kX * (distance - TARGET_DISTANCE_METERS); // forward/backward
    double ySpeed = -kY * (tx / 30.0); // small strafe correction if tag appears off-center in camera

    // clamp speeds
    rot = clamp(rot, -1, 1);
    xSpeed = clamp(xSpeed, -1, 1);
    ySpeed = clamp(ySpeed, -1, 1);

    // --- Drive robot (robot-relative) ---
    m_drive.drive(xSpeed, ySpeed, rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
    m_drive.setPattern(-0.75);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!LimelightHelpers.getTV("limelight")) return false;

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");
    double totalAngleDeg = CAM_MOUNT_ANGLE_DEGREES + ty;
    double totalAngleRad = Math.toRadians(totalAngleDeg);
    double distance = (TAG_HEIGHT_METERS - CAM_HEIGHT_METERS) / Math.tan(totalAngleRad);
    //double distance = m_lidarSubsystem.getDistance();
    double distanceError = distance - TARGET_DISTANCE_METERS;

    // finishes when robot is roughly centered and at target distance
    return Math.abs(tx) < 1.0 && Math.abs(distanceError) < 0.05;
  }

  private double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
