// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Utils {
  public static void copyModuleStates(SwerveModuleState[] copier, SwerveModuleState[] reciever) {
    for (int i = 0; i < copier.length; i++) {
      reciever[i] = new SwerveModuleState(copier[i].speedMetersPerSecond, copier[i].angle);
    }
  }

  public static double getAdjustedYawDegrees(double initialvalue, double addedValue) {
    double numTo180 = 180 - addedValue;

    return (initialvalue + numTo180) % 360 < 0
        ? ((initialvalue + numTo180) % 360) + 360.0
        : ((initialvalue + numTo180) % 360);
  }

  /**
   *
   *
   * <h3>ModifyInputs</h3>
   *
   * Returns the input to the power of the modifier
   *
   * @param input Input to modify
   * @param modifier Puts the input to the power of this
   */
  public static double modifyInputs(double input, double modifier) {
    return input >= 0 ? Math.pow(input, modifier) : -Math.pow(-input, modifier);
  }
}
