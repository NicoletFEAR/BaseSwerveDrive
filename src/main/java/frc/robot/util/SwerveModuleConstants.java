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
  public double offset;

  public SwerveModuleConstants(int driveId, int steerId, int steerEncoderId, double offset) {
    this.driveId = driveId;
    this.steerId = steerId;
    this.steerEncoderId = steerEncoderId;
    this.offset = offset;
  }
}
