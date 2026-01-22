package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConfig.VisionTrackingMode;
import frc.robot.RobotContainer;

public class VisionCommands {

  // final static VisionSubsystem frontVisionSubsystem = RobotContainer.frontVisionSubsystem;
  // final static VisionSubsystem rearVisionSubsystem = RobotContainer.rearVisionSubsystem;

  // public static Command trackReef() {
  //   return Commands.runOnce(() -> {
  //     frontVisionSubsystem.stopTracking();
  //     rearVisionSubsystem.startTracking(VisionSubsystem.VisionTarget.ReefApriltag);
  //     RobotContainer.visionTrackingMode = VisionTrackingMode.Rear;
  //   });
  // }

  // public static Command trackLoadStation() {
  //   return Commands.runOnce(() -> {
  //     rearVisionSubsystem.stopTracking();
  //     frontVisionSubsystem.startTracking(VisionSubsystem.VisionTarget.LoadStationAprilTag);
  //     RobotContainer.visionTrackingMode = VisionTrackingMode.Front;
  //   });
  // }

  // public static Command trackBarge() {
  //   return Commands.runOnce(() -> {
  //     rearVisionSubsystem.stopTracking();
  //     frontVisionSubsystem.startTracking(VisionSubsystem.VisionTarget.BargeAprilTag);
  //     RobotContainer.visionTrackingMode = VisionTrackingMode.Front;
  //   });
  // }

  // public static Command trackProcessor() {
  //   return Commands.runOnce(() -> {
  //     rearVisionSubsystem.stopTracking();
  //     frontVisionSubsystem.startTracking(VisionSubsystem.VisionTarget.ProcessorAprilTag);
  //     RobotContainer.visionTrackingMode = VisionTrackingMode.Front;
  //   });
  // }
}
