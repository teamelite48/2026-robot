// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.limelights;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.components.limelights.lib.LimelightCameraConfig;

public class TargetBasedShooting {
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

    final LimelightCameraConfig config;

    public TargetBasedShooting(LimelightCameraConfig limelight) {
        this.config = limelight;

        final NetworkTable table = NetworkTableInstance.getDefault().getTable(config.hostname);

        this.tid = table.getEntry("tid");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tv = table.getEntry("tv");
        this.ledMode = table.getEntry("ledMode");
        this.pipeline = table.getEntry("pipeline");
        this.botpose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

        enableLed(false);
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
        return hasTarget() && Math.abs(getXOffsetDegrees()) < config.has_target_degrees_threshold;
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

        double angleDegrees = config.mount_pitch + getYOffsetDegrees();
        double angleRadians = Math.toRadians(angleDegrees);

        double heightDifferenceInches = config.april_tag_height_inches - config.mount_height_inches;

        if (Math.abs(Math.tan(angleRadians)) < 1e-6) {
            return;
        }

        double distanceInches = heightDifferenceInches / Math.tan(angleRadians);

        distanceFromTargetFeet = distanceInches / 12.0;
    }
}
