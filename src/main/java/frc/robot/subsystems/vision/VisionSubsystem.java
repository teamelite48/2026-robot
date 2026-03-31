// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.components.limelights.PoseBasedShooting;
import frc.robot.components.limelights.TargetBasedShooting;
import frc.robot.components.limelights.lib.LimelightCamera;
import frc.robot.subsystems.drive.DriveSubsystem;

import static frc.robot.subsystems.vision.VisionConfig.*;


public class VisionSubsystem extends SubsystemBase {

  public enum VisionMode {
    TURRET_ONLY,
    POSE_BASED_DUAL
  }

  private final DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

  // private VisionTarget target;
  // private Optional<Alliance> alliance = Optional.empty();

  // private boolean isTracking = false;
  // private double feetFromTarget;
  // private double distanceFromTargetFeet;

  private final LimelightCamera turretLimelight;
  private final LimelightCamera leftLimelight;
  private final LimelightCamera rightLimelight;
  private VisionMode currentMode = VisionMode.TURRET_ONLY;
  private final SwerveDrivePoseEstimator poseEstimator = driveSubsystem.poseEstimator;

  public VisionSubsystem() {

    var turretConfig = getTurretLimelightConfig();
    var leftConfig = getLeftLimelightConfig();
    var rightConfig = getRightLimelightConfig();

    this.turretLimelight = new TargetBasedShooting(turretConfig);
    this.leftLimelight = new PoseBasedShooting(leftConfig);
    this.rightLimelight = new PoseBasedShooting(rightConfig);

    initDashboard(turretLimelight.getHostname());
  }

  @Override
  public void periodic() {

    // if (!hasTarget()) {
    //   return;
    // }

    if (leftLimelight.hasTarget()) {
        Pose2d pose = leftLimelight.getBotPose(); // TODO: Figure out this method and where to put it.
        double timestamp = Timer.getFPGATimestamp() - leftLimelight.getLatency();

        addVisionMeasurement(pose, timestamp);
    }

    if (rightLimelight.hasTarget()) {
        Pose2d pose = rightLimelight.getBotPose(); // TODO: Figure out this method and where to put it.
        double timestamp = Timer.getFPGATimestamp() - rightLimelight.getLatency();

        addVisionMeasurement(pose, timestamp);
    }

    // SANITY CHECK: If distance is more than 1.5 meters away, then ignore it. (I don't know if this is actually a problem. -Trevor)

    // if (visionPose.getTranslation().getDistance(getPose().getTranslation()) < 1.5) {
    //     poseEstimator.addVisionMeasurement(visionPose, timestamp);
    // }
  }

  public void setVisionMode(VisionMode mode) {
    this.currentMode = mode;
  }

  /**
   * Logic to decide which camera data to return.
   * If in Pose mode, it prioritizes the camera that currently sees a target.
   */
  private LimelightCamera getActiveCamera() {

    if (currentMode == VisionMode.TURRET_ONLY) {
      return turretLimelight;
    }
    else {
      // Dual camera logic: return whichever has a better target,
      // or default to left if both see one.
      if (leftLimelight.hasTarget()) return leftLimelight;
      return rightLimelight;
    }
  }

  public long getTargetId() {
    return getActiveCamera().getTargetId();
  }

  public double getFeetFromTarget() {
    LimelightCamera active = getActiveCamera();
    if (active instanceof TargetBasedShooting) {
      return ((TargetBasedShooting) active).getFeetFromTarget();
    }
    else {
      return ((PoseBasedShooting) active).getFeetFromTarget();
    }
  }

  public boolean hasTarget() {
    return getActiveCamera().hasTarget();
  }

  public boolean hasTargetWithinParameters() {
    return hasTarget() && Math.abs(getXOffsetDegrees()) < HAS_TARGET_DEGREES_THRESHOLD;
  }

  public double getXOffsetDegrees() {
    return getActiveCamera().getXOffsetDegrees();
  }

  private double getYOffsetDegrees() {
    // TODO: negative when target is below cross hair, positive when it's above
    return -getActiveCamera().getYOffsetDegrees();
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
      poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  private void initDashboard(String tabName) {

    var tab = Shuffleboard.getTab("Vision");

    tab.addDouble("Target ID", () -> getTargetId())
      .withPosition(0, 0);

    tab.addDouble("X Offset (deg)", () -> getXOffsetDegrees())
      .withPosition(1, 0);

    tab.addDouble("Y  Offset (deg)", () -> getYOffsetDegrees())
      .withPosition(2, 0);

    // tab.addInteger("LED Mode", () -> ledMode.getInteger(0))
    //   .withPosition(3, 0);

    tab.addDouble("Distance from Target (ft)", () -> getFeetFromTarget())
      .withPosition(0, 1)
      .withSize(2, 1);

    tab.addString("Vision Mode", () -> currentMode.toString())
      .withPosition(3, 0);

    // tab.addBoolean("Tracking Target", () -> isTracking)
    //   .withPosition(3, 1);

    // tab.addString("Vision Target", () -> getTargetName())
    //   .withPosition(4, 1);

    tab.addBoolean("Has Target", () -> hasTarget());
    // tab.addString("Alliance", () -> (alliance.isPresent() ? alliance.get() : "Not Present").toString());
    // tab.addDoubleArray("botpose", () -> botpose);
  }
}
