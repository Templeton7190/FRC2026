// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarSubsystem extends SubsystemBase {
  private final I2C lidar;
  private static final int LIDAR_ADDR = 0x62;
  private static final byte ACQ_COMMAND = 0x00;
  private static final byte MEASURE = 0x04;
  private static final byte DISTANCE_REG = (byte) 0x8f;

  public LidarSubsystem() {
      lidar = new I2C(I2C.Port.kOnboard, LIDAR_ADDR);
  }

  public double getDistanceMeters() {
      byte[] buffer = new byte[2];

      // Trigger new measurement
      lidar.write(ACQ_COMMAND, MEASURE);
      Timer.delay(0.06); // give it time

      System.out.println("LIDAR connected: " + lidar.addressOnly());

      boolean failed = lidar.read(0x8f, 2, buffer);
      if (!failed) {
          int distanceCm = ((buffer[0] & 0xFF) << 8) | (buffer[1] & 0xFF);
          double distanceM = distanceCm / 10000.0;
          SmartDashboard.putNumber("LIDAR Raw High", buffer[0] & 0xFF);
          SmartDashboard.putNumber("LIDAR Raw Low", buffer[1] & 0xFF);
          SmartDashboard.putNumber("LIDAR Distance (m)", distanceM);
          return distanceM;
      } else {
          SmartDashboard.putString("LIDAR ERROR", "Read failed");
          return -1;
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
