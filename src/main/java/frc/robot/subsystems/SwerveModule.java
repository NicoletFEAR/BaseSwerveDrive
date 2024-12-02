// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.DeviceConfigurator;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {

  SwerveModuleConstants m_constants;

  private CANSparkMax m_steerMotor;
  private CANSparkFlex m_driveMotor;
  private AbsoluteEncoder m_steerAbsEncoder;

  private RelativeEncoder m_steerEncoder;
  private RelativeEncoder m_driveEncoder;

  private SparkPIDController m_steerController;
  private SparkPIDController m_driveController;

  private SwerveModulePosition m_modulePosition;
  private SwerveModuleState m_moduleState;

  private double m_lastSpeed;
  private double m_lastAngle;

  private Rotation2d m_simAngle = new Rotation2d();
  private double m_simDist;
  private double m_simVel;

  public SwerveModule(SwerveModuleConstants constants) {
    m_constants = constants;

    m_steerMotor = new CANSparkMax(constants.steerId, MotorType.kBrushless);
    m_driveMotor = new CANSparkFlex(constants.driveId, MotorType.kBrushless);

    m_steerEncoder = m_steerMotor.getEncoder();
    m_driveEncoder = m_driveMotor.getEncoder();

    m_steerController = m_steerMotor.getPIDController();
    m_driveController = m_driveMotor.getPIDController();

    m_steerAbsEncoder = m_steerMotor.getAbsoluteEncoder();

    m_modulePosition = new SwerveModulePosition();
    m_moduleState = new SwerveModuleState();

    DeviceConfigurator.configureSparkMaxSteerMotor(m_steerMotor);
    DeviceConfigurator.configureSparkFlexDriveMotor(m_driveMotor);
  }

  public void resetAngleToAbsolute() {
    m_steerEncoder.setPosition((m_steerAbsEncoder.getPosition() * 360));
  }

  public void burnFlash() {
    m_driveMotor.burnFlash();
    m_steerMotor.burnFlash();
  }

  public Rotation2d getModuleHeading() {
    return m_modulePosition.angle;
  }

  public double getAbsolutePosition() {
    return m_steerAbsEncoder.getPosition();
  }

  public SwerveModulePosition getModulePosition() {
    if (RobotBase.isReal()) {
      m_modulePosition.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
      m_modulePosition.distanceMeters = m_driveEncoder.getPosition();
    } else {
      m_modulePosition.angle = m_simAngle;
      m_modulePosition.distanceMeters = m_simDist;
    }
    return m_modulePosition;
  }

  public SwerveModuleState getModuleState() {
    if (RobotBase.isReal()) {
      m_moduleState.angle = Rotation2d.fromDegrees(m_steerEncoder.getPosition());
      m_moduleState.speedMetersPerSecond = m_driveEncoder.getVelocity();
    } else {
      m_moduleState.angle = m_simAngle;
      m_moduleState.speedMetersPerSecond = m_simVel;
    }
    return m_moduleState;
  }

  public void setSwerveModuleState(SwerveModuleState moduleState, boolean isOpenLoop) {

    moduleState = SwerveModuleState.optimize(moduleState, getModuleHeading());

    moduleState.speedMetersPerSecond *= moduleState.angle.minus(getModuleHeading()).getCos();

    if (moduleState.angle.getDegrees() != m_lastAngle) {
      m_steerController.setReference(moduleState.angle.getDegrees(), ControlType.kPosition);
      m_lastAngle = moduleState.angle.getDegrees();
    }

    if (moduleState.speedMetersPerSecond != m_lastSpeed) {
      if (isOpenLoop) {
        m_driveMotor.set(moduleState.speedMetersPerSecond / DriveConstants.kMaxModuleSpeed);
      } else {
        m_driveController.setReference(moduleState.speedMetersPerSecond, ControlType.kVelocity);
      }
      m_lastSpeed = moduleState.speedMetersPerSecond;
    }

    if (RobotBase.isSimulation()) {
      m_simAngle = moduleState.angle;
      m_simVel = moduleState.speedMetersPerSecond;
      m_simDist += moduleState.speedMetersPerSecond / (1 / Constants.kdt);
    }
  }
}
