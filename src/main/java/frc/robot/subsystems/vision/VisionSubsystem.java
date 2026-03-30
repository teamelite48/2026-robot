// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.vision.VisionConfig.*;

import java.util.Optional;


public class VisionSubsystem extends SubsystemBase {

  // public enum VisionTarget {

  //   HubApriltag(0, HUB_APRILTAG_HEIGHT_INCHES);
  //   // LoadStationAprilTag(0, LOAD_STATION_APRILTAG_HEIGHT_INCHES),
  //   // BargeAprilTag(0, BARGE_APRILTAG_HEIGHT_INCHES),
  //   // ProcessorAprilTag(0, PROCESSOR_APRILTAG_HEIGHT_INCHES);

  //   public final int pipelineId;
  //   public final double heightInInches;

  //   private VisionTarget(int pipelineId, double targetInInches) {
  //     this.pipelineId = pipelineId;
  //     this.heightInInches = targetInInches;
  //   }


  final NetworkTableEntry tid;
  final NetworkTableEntry tx;
  final NetworkTableEntry ty;
  final NetworkTableEntry ta;
  final NetworkTableEntry tv;
  final NetworkTableEntry ledMode;
  final NetworkTableEntry pipeline;
  final double[] botpose;

  // private VisionTarget target;
  private Optional<Alliance> alliance = Optional.empty();

  private boolean isTracking = false;
  private double feetFromTarget;
  private double distanceFromTargetFeet;

  public VisionSubsystem(String limelightName) {
    // limielightName = hostname in Limelight settings
    // stopTracking();

    final NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);

    this.tid = table.getEntry("tid");
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
    this.ledMode = table.getEntry("ledMode");
    this.pipeline = table.getEntry("pipeline");
    this.botpose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

    enableLed(false);
    initDashboard(limelightName);
  }

  @Override
  public void periodic() {

    if (!hasTarget()) {
      return;
    }

    updateDistanceToTarget();
  }

  public void enableLed(boolean isEnabled) {
    if (isEnabled) {
      ledMode.setNumber(3);
    }
    else {
      ledMode.setNumber(1);
    }
  }

  public long getTargetId() {
    return tid.getInteger(0);
  }

  // IF LOOKING AT APRILTAGS 9, 10, 25, 26 AND YOUR X OFFSET IS POSITIVE THEN BIAS TURRET RIGHT TO SOME DEGREE
  // IT'S GONNA TAKE TRIG
  // FOR NOW
  // IF X OFFSET IS > 0 THEN AFTER CALCULATION ADD DEGREES
  // IF DISTANCE IS LESS THAN 8 FEET AWAY THEN ADD 2 DEGREES
  // IF DISTANCE IS GREATER THAN 8 FEET AWAY THEN ADD 1 DEGREE

  // public void offsetTurret() {
  //   long idNumber = getTargetId();


  //   }
  // }

  // public void startTracking(VisionTarget target) {
  //   this.target = target;
  //   isTracking = true;
  //   enableLed(true);
  // }

  // public void stopTracking() {
  //   isTracking = false;
  //   enableLed(false);
  // }

  public double getFeetFromTarget(){
    return distanceFromTargetFeet;
  }

  public boolean hasTarget() {
    return tv.getDouble(0) == 1 ? true: false;
  }

  public boolean hasTargetWithinParameters() {
    return hasTarget() && Math.abs(getXOffsetDegrees()) < HAS_TARGET_DEGREES_THRESHOLD;
  }

  public double getXOffsetDegrees() {
    return tx.getDouble(0.0);
  }

  private double getYOffsetDegrees() {
    // TODO: negative when target is below cross hair, positive when it's above
    return -ty.getDouble(0.0);
  }

  // private void updateFeetFromTarget() {

  //   double degreesToApriltag = MOUNT_ANGLE_DEGREES + getYOffsetDegrees();
  //   double radiansToApriltag = degreesToApriltag * (Math.PI / 180.0);

  //   feetFromTarget = (((HUB_APRILTAG_HEIGHT_INCHES - MOUNT_HEIGHT_INCHES) / Math.tan(radiansToApriltag)) / 12.0);
  // }

  private void updateDistanceToTarget() {

    double angleDegrees = TURRET_MOUNT_ANGLE_DEGREES + getYOffsetDegrees();
    double angleRadians = Math.toRadians(angleDegrees);

    double heightDifferenceInches = HUB_APRILTAG_HEIGHT_INCHES - TURRET_MOUNT_HEIGHT_INCHES;

    if (Math.abs(Math.tan(angleRadians)) < 1e-6) {
        return;
    }

    double distanceInches = heightDifferenceInches / Math.tan(angleRadians);

    distanceFromTargetFeet = distanceInches / 12.0;
  }

  private void initDashboard(String tabName) {

    var tab = Shuffleboard.getTab("Vision");

    tab.addDouble("Target ID", () -> getTargetId())
      .withPosition(0, 0);

    tab.addDouble("X Offset (deg)", () -> getXOffsetDegrees())
      .withPosition(1, 0);

    tab.addDouble("Y  Offset (deg)", () -> getYOffsetDegrees())
      .withPosition(2, 0);

    tab.addInteger("LED Mode", () -> ledMode.getInteger(0))
      .withPosition(3, 0);

    tab.addDouble("Distance from Target (ft)", () -> getFeetFromTarget())
      .withPosition(0, 1)
      .withSize(2, 1);

    tab.addBoolean("Tracking Target", () -> isTracking)
      .withPosition(3, 1);

    // tab.addString("Vision Target", () -> getTargetName())
    //   .withPosition(4, 1);

    tab.addBoolean("Has Target", () -> hasTarget());
    tab.addString("Alliance", () -> (alliance.isPresent() ? alliance.get() : "Not Present").toString());
    tab.addDoubleArray("botpose", () -> botpose);
  }

  // public String getTargetName() {
  //   return target == null
  //     ? "None"
  //     : target.name();
  // }
}
