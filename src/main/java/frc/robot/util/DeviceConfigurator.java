// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.DriveConstants;

public class DeviceConfigurator {
  public static void configureSparkMaxSteerMotor(CANSparkMax motor) {
    RelativeEncoder encoder = motor.getEncoder();
    SparkPIDController controller = motor.getPIDController();

    motor.restoreFactoryDefaults();

    motor.setInverted(true);
    motor.setSmartCurrentLimit(40);
    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(DriveConstants.kTurnRotationsToDegrees);

    encoder.setPosition(0);

    controller.setP(DriveConstants.turnkp);
    controller.setI(DriveConstants.turnki);
    controller.setD(DriveConstants.turnkd);
    controller.setFF(DriveConstants.turnkff);
  }

  public static void configureSparkFlexDriveMotor(CANSparkMax motor) {
    RelativeEncoder encoder = motor.getEncoder();
    SparkPIDController controller = motor.getPIDController();

    motor.restoreFactoryDefaults();

    motor.setInverted(true);
    motor.setSmartCurrentLimit(80);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setOpenLoopRampRate(DriveConstants.driverampRate);

    encoder.setPositionConversionFactor(DriveConstants.kDriveRevToMeters);
    encoder.setVelocityConversionFactor(DriveConstants.kDriveRpmToMetersPerSecond);
    encoder.setPosition(0);

    controller.setP(DriveConstants.drivekp);
    controller.setI(DriveConstants.driveki);
    controller.setD(DriveConstants.drivekd);
    controller.setFF(DriveConstants.drivekff);
  }

  public static void configureCANcoder(CANcoder encoder, double offset) {
    CANcoderConfiguration configuration = new CANcoderConfiguration();

    encoder.getConfigurator().apply(configuration);

    configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    configuration.MagnetSensor.MagnetOffset = offset;
    configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    encoder.getConfigurator().apply(configuration);
  }
}
