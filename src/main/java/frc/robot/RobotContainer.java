package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.RobotConfig.GamePiece;
import frc.robot.RobotConfig.VisionTrackingMode;
import frc.robot.commands.Robot.RobotCommands;
import frc.robot.controls.DualShock4Controller;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.Gear;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.turret.TurretCommands;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  // public static VisionSubsystem rearVisionSubsystem = new VisionSubsystem("limelight-rear");
  // public static VisionSubsystem frontVisionSubsystem = new VisionSubsystem("limelight-front");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  public static ClimberSubsystem climberSubsytem = new ClimberSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  public static GamePiece gamePieceMode = GamePiece.Coral;
  public static VisionTrackingMode visionTrackingMode = VisionTrackingMode.Rear;
  public static boolean isWristFlippable = false;

  public static CameraServer camera;
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();

  public RobotContainer() {

    autoChooser = RobotContainer.initAutoChooser();

    // CameraServer.startAutomaticCapture();

    driveSubsystem
        .setDefaultCommand(DriveCommands.drive(() -> pilotController.getLeftAxes(), () -> pilotController.getRightAxes()));

    bindPilotControls();
    bindCopilotControls();
    bindTestControls();

    initDashboard();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void bindPilotControls() {
    pilotController.ps.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro(), driveSubsystem));

    
  }

  private void bindCopilotControls() {

    
  }

  private void bindTestControls() {
    testController.left
      .whileTrue(TurretCommands.RotateTurretCounterClockwise())
      .onFalse(TurretCommands.stop());
    testController.right
      .whileTrue(TurretCommands.RotateTurretClockwise())
      .onFalse(TurretCommands.stop());
  }

  public static SendableChooser<Command> initAutoChooser() {

    NamedCommands.registerCommand("Seek Target", DriveCommands.seekTarget());
    

    var autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    return autoChooser;
  }

  public void initDashboard() {
    var robotContainerTab = Shuffleboard.getTab("Robot Container");

    // robotContainerTab.addString("Game Piece Mode", () -> gamePieceMode.toString())
    //   .withPosition(0, 0)
    //   .withSize(2, 1);

    // robotContainerTab.addString("Vision Tracking Mode", () -> RobotContainer.visionTrackingMode.toString())
    //   .withPosition(2, 0)
    //   .withSize(2, 1);

    // robotContainerTab.addString("Current Vision Target", () -> {
    //   return RobotContainer.visionTrackingMode == VisionTrackingMode.Front
    //     ? frontVisionSubsystem.getTargetName()
    //     : rearVisionSubsystem.getTargetName();
    // })
    //   .withPosition(2, 1)
    //   .withSize(2, 1);

    // robotContainerTab.addBoolean("Is Wrist Flippable", () -> RobotContainer.isWristFlippable)
    //   .withPosition(4, 0)
    //   .withSize(1, 1);
  }
}
