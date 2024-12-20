// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // inputs.connected = navX.isConnected();
    // inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    // inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    // inputs.odometryYawTimestamps =
    //     yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryYawPositions =
    //     yawPositionQueue.stream()
    //         .map((Double value) -> Rotation2d.fromDegrees(-value))
    //         .toArray(Rotation2d[]::new);
    // yawTimestampQueue.clear();
    // yawPositionQueue.clear();
  }
}