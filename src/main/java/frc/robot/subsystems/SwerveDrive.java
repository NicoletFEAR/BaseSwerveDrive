// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team6328.swerve.SwerveSetpoint;
import frc.lib.team6328.swerve.SwerveSetpointGenerator;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.util.Utils;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive m_instance = null;

  private SwerveModule[] m_modules;

  private Pigeon2 m_pigeon;

  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_poseEstimator;
  private SwerveSetpointGenerator m_setpointGenerator;

  private SwerveModuleState[] m_desiredModuleStates;
  private SwerveModulePosition[] m_modulePositions;
  private ChassisSpeeds m_chassisSpeeds;
  private SwerveSetpoint m_optimizedSetpoint;

  private DriveMode m_driveMode = DriveMode.TELEOP;

  private Field2d m_field;

  private double m_simYaw;

  public SwerveDrive() {
    m_modules =
        new SwerveModule[] {
          new SwerveModule(DriveConstants.kFrontLeft),
          new SwerveModule(DriveConstants.kFrontRight),
          new SwerveModule(DriveConstants.kBackLeft),
          new SwerveModule(DriveConstants.kBackRight)
        };

    m_pigeon = new Pigeon2(DriveConstants.kPigeonId);

    m_kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);

    m_desiredModuleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    m_modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    m_setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(m_kinematics)
            .moduleLocations(DriveConstants.kModuleTranslations)
            .build();

    m_optimizedSetpoint = new SwerveSetpoint(new ChassisSpeeds(), m_desiredModuleStates);
    m_chassisSpeeds = new ChassisSpeeds();

    Timer.delay(1.0);
    resetModulesToAbsolute();
    burnModuleFlashes();

    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.0),
            VecBuilder.fill(0.9, 0.9, 9999999));

    m_pigeon.setYaw(0);

    m_field = new Field2d();
    RobotContainer.m_mainTab.add(m_field);

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxModuleSpeed, // Max module speed, in m/s
            DriveConstants
                .kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  public static SwerveDrive getInstance() {
    if (m_instance == null) {
      m_instance = new SwerveDrive();
    }

    return m_instance;
  }

  public SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < m_modules.length; i++) {
      m_modulePositions[i] = m_modules[i].getModulePosition();
    }

    return m_modulePositions;
  }

  public SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < m_modules.length; i++) {
      m_desiredModuleStates[i] = m_modules[i].getModuleState();
    }

    return m_desiredModuleStates;
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(m_chassisSpeeds, getYaw());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setModuleStates(m_kinematics.toSwerveModuleStates(speeds), false);
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule module : m_modules) {
      module.resetAngleToAbsolute();
    }
  }

  public void burnModuleFlashes() {
    for (SwerveModule module : m_modules) {
      module.burnFlash();
    }
  }

  public void setModuleStates(boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_desiredModuleStates, DriveConstants.kMaxModuleSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setSwerveModuleState(m_desiredModuleStates[i], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] moduleStates, boolean isOpenLoop) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setSwerveModuleState(moduleStates[i], isOpenLoop);
    }
  }

  public Rotation2d getYaw() {
    if (RobotBase.isReal()) {
      return Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
    } else {
      return Rotation2d.fromDegrees(m_simYaw);
    }
  }

  public void setDriveMode(DriveMode driveMode) {
    m_driveMode = driveMode;
  }

  public void drive(
      double throttle, double strafe, double steer, boolean isOpenLoop, boolean isFieldRelative) {

    if (throttle + strafe + steer != 0 && m_driveMode == DriveMode.XWHEELS) {
      m_driveMode = DriveMode.TELEOP;
    }

    switch (m_driveMode) {
      case TELEOP:
        throttle *= DriveConstants.kMaxModuleSpeed;
        strafe *= DriveConstants.kMaxModuleSpeed;
        steer *= DriveConstants.kMaxModuleSpeed;

        m_chassisSpeeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, steer, getYaw())
                : new ChassisSpeeds(throttle, strafe, steer);

        m_chassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, Constants.kdt);

        m_desiredModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        m_optimizedSetpoint =
            m_setpointGenerator.generateSetpoint(
                DriveConstants.kModuleLimits, m_optimizedSetpoint, m_chassisSpeeds, Constants.kdt);

        if (isOpenLoop) {
          setModuleStates(m_optimizedSetpoint.moduleStates(), isOpenLoop);
        } else {
          setModuleStates(m_desiredModuleStates, isOpenLoop);
        }

        if (RobotBase.isSimulation())
          m_simYaw += Units.radiansToDegrees(m_chassisSpeeds.omegaRadiansPerSecond * Constants.kdt);

        break;
      case XWHEELS:
        Utils.copyModuleStates(DriveConstants.kXWheels, m_desiredModuleStates);
        setModuleStates(isOpenLoop);
        break;
    }
  }

  public void runVolts(Measure<Voltage> volts) {
    for (SwerveModule module : m_modules) {
      module.runVolts(volts, 0);
    }
  }

  public Command characterizeDrivebase(BooleanSupplier finishRoutine) {
    var sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage), null, this));

    return new SequentialCommandGroup(
        new PrintCommand("Starting"),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(finishRoutine),
        new WaitCommand(1.0),
        new PrintCommand("Starting"),
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(finishRoutine),
        new PrintCommand("Starting"),
        new WaitCommand(1.0),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(finishRoutine),
        new PrintCommand("Starting"),
        new WaitCommand(1.0),
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(finishRoutine));
  }

  @Override
  public void periodic() {
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

    m_field.getRobotObject().setPose(getPose());

    Logger.recordOutput("Swerve/Pose", m_poseEstimator.getEstimatedPosition());
    Logger.recordOutput("Swerve/Module Positions", getModulePositions());
    Logger.recordOutput("Swerve/Module States", getModuleStates());
    Logger.recordOutput("Swerve/Chassis Speeds", m_chassisSpeeds);
    Logger.recordOutput("Swerve/Robot Relative Chassis Speeds", getRobotRelativeSpeeds());
    Logger.recordOutput(
        "Swerve/Optimized Desired Module States", m_optimizedSetpoint.moduleStates());
    Logger.recordOutput("Swerve/Desired Module States", m_desiredModuleStates);
  }

  public enum DriveMode {
    TELEOP,
    XWHEELS
  }
}
