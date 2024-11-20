// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class SwerveModuleConstants {
  public int driveId;
  public int steerId;
  public int steerEncoderId;

  public int steerEncoderPin1;
  public int steerEncoderPin2;
  public int driveEncoderPin1;
  public int driveEncoderPin2;

  public double offset;

  public SwerveModuleConstants(int driveId, int steerId, int steerEncoderId, double offset, int steerEncoderPin1,
                              int steerEncoderPin2, int driveEncoderPin1, int driveEncoderPin2) {
    this.driveId = driveId;
    this.steerId = steerId;
    this.steerEncoderId = steerEncoderId;

    this.steerEncoderPin1 = steerEncoderPin1;
    this.steerEncoderPin2 = steerEncoderPin2;
    this.driveEncoderPin1 = driveEncoderPin1;
    this.driveEncoderPin2 = driveEncoderPin1;

    this.offset = offset;
  }
}
