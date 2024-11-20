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

  public int steerEncoderPin;
  public int driveEncoderPin;
  public double steerEncoderRotDis;
  public double driveEncoderRotDis;

  public double offset;

  public SwerveModuleConstants(int driveId, int steerId, int steerEncoderId, double offset, int steerEncoderPin,
                              int driveEncoderPin, double steerEncoderRotDis, double driveEncoderPin) {
    this.driveId = driveId;
    this.steerId = steerId;
    this.steerEncoderId = steerEncoderId;

    this.steerEncoderPin = steerEncoderPin;
    this.driveEncoderPin = driveEncoderPin;
    this.steerEncoderRotDis = steerEncoderRotDis;
    this.driveEncoderRotDis = driveEncoderRotDis;

    this.offset = offset;
  }
}
