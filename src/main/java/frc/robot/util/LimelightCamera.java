package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;

public class LimelightCamera {

  private String m_cameraName;

  private final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();

  public LimelightCamera(String cameraName) {
    m_cameraName = cameraName;
  }

  public void addPoseEstimate(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds robotSpeeds) {

    PoseEstimate poseEstimate = getBotPoseEstimate_wpiBlue_MegaTag2();

    Pose2d pose = poseEstimate.pose;

    boolean isPoseInField =
        pose.getTranslation().getX() < Constants.kFieldTopRight.getX()
            && pose.getTranslation().getY() < Constants.kFieldTopRight.getY()
            && !pose.getTranslation().equals(new Translation2d(0, 0));
    boolean isWithinTolerance =
        pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation())
            < VisionConstants.kOffsetTolerance;

    Logger.recordOutput("Vision/" + m_cameraName + "/isPoseInField", isPoseInField);
    Logger.recordOutput("Vision/" + m_cameraName + "/isWithinTolerance", isWithinTolerance);

    double tagAmountTrust = VisionConstants.kTargetAmountConstant * poseEstimate.tagCount;
    double speedTrust =
        Math.sqrt(
                Math.pow(robotSpeeds.vxMetersPerSecond, 2)
                    + Math.pow(robotSpeeds.vyMetersPerSecond, 2))
            * VisionConstants.kSpeedsConstant;
    double rotationTrust = robotSpeeds.omegaRadiansPerSecond * VisionConstants.kRotationsConstant;
    double distanceTrust = poseEstimate.avgTagDist * VisionConstants.kDistanceConstant;
    double areaTrust = poseEstimate.avgTagDist * VisionConstants.kAreaConstant;

    Logger.recordOutput("Vision/" + m_cameraName + "/tagAmountTrust", tagAmountTrust);
    Logger.recordOutput("Vision/" + m_cameraName + "/speedTrust", speedTrust);
    Logger.recordOutput("Vision/" + m_cameraName + "/rotationTrust", rotationTrust);
    Logger.recordOutput("Vision/" + m_cameraName + "/distanceTrust", distanceTrust);
    Logger.recordOutput("Vision/" + m_cameraName + "/areaTrust", areaTrust);

    double trust =
        Math.max(speedTrust + rotationTrust - tagAmountTrust + distanceTrust - areaTrust, 0.1);

    Matrix<N3, N1> poseMatrix = VecBuilder.fill(trust, trust, 9999999);

    if (isPoseInField && isWithinTolerance) {
      poseEstimator.addVisionMeasurement(pose, poseEstimate.timestampSeconds, poseMatrix);
      Logger.recordOutput("Vision/" + m_cameraName + "/Trust", trust);
    } else {
      Logger.recordOutput("Vision/" + m_cameraName + "/Trust", 0);
    }
  }

  public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
    return getBotPoseEstimate("botpose_orb_wpiblue", true);
  }

  public void SetRobotOrientation(
      double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
    SetRobotOrientation_INTERNAL(yaw, yawRate, pitch, pitchRate, roll, rollRate, true);
  }

  private void SetRobotOrientation_INTERNAL(
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate,
      boolean flush) {

    double[] entries = new double[6];
    entries[0] = yaw;
    entries[1] = yawRate;
    entries[2] = pitch;
    entries[3] = pitchRate;
    entries[4] = roll;
    entries[5] = rollRate;
    setLimelightNTDoubleArray(m_cameraName, "robot_orientation_set", entries);
    if (flush) {
      Flush();
    }
  }

  public void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public void Flush() {
    NetworkTableInstance.getDefault().flush();
  }

  public NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  private String sanitizeName(String name) {
    if (name == "" || name == null) {
      return "limelight";
    }
    return name;
  }

  public NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
    String key = tableName + "/" + entryName;
    return doubleArrayEntries.computeIfAbsent(
        key,
        k -> {
          NetworkTable table = getLimelightNTTable(tableName);
          return table.getDoubleArrayTopic(entryName).getEntry(new double[0]);
        });
  }

  private double extractArrayEntry(double[] inData, int position) {
    if (inData.length < position + 1) {
      return 0;
    }
    return inData[position];
  }

  private PoseEstimate getBotPoseEstimate(String entryName, boolean isMegaTag2) {
    DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(m_cameraName, entryName);

    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) {
      return null;
    }

    var pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);

    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    RawFiducial[] rawFiducials = new RawFiducial[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        double txnc = poseArray[baseIndex + 1];
        double tync = poseArray[baseIndex + 2];
        double ta = poseArray[baseIndex + 3];
        double distToCamera = poseArray[baseIndex + 4];
        double distToRobot = poseArray[baseIndex + 5];
        double ambiguity = poseArray[baseIndex + 6];
        rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }

    return new PoseEstimate(
        pose,
        adjustedTimestamp,
        latency,
        tagCount,
        tagSpan,
        tagDist,
        tagArea,
        rawFiducials,
        isMegaTag2);
  }

  public class RawFiducial {
    public int id = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public double distToCamera = 0;
    public double distToRobot = 0;
    public double ambiguity = 0;

    public RawFiducial(
        int id,
        double txnc,
        double tync,
        double ta,
        double distToCamera,
        double distToRobot,
        double ambiguity) {
      this.id = id;
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
    }
  }

  public class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;

    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    public PoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        RawFiducial[] rawFiducials,
        boolean isMegaTag2) {

      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.latency = latency;
      this.tagCount = tagCount;
      this.tagSpan = tagSpan;
      this.avgTagDist = avgTagDist;
      this.avgTagArea = avgTagArea;
      this.rawFiducials = rawFiducials;
      this.isMegaTag2 = isMegaTag2;
    }
  }
}
