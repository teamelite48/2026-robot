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
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.turret.TurretCommands;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  // public static VisionSubsystem collectorVisionSubsystem = new VisionSubsystem("limelight-collector");
  public static VisionSubsystem shooterVisionSubsystem = new VisionSubsystem("limelight-shooter");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  public static ClimberSubsystem climberSubsytem = new ClimberSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final SendableChooser<Command> autoChooser;

  public static boolean isAimAssistEnabled = true;
  //public static GamePiece gamePieceMode = GamePiece.Coral;
  public static VisionTrackingMode visionTrackingMode = VisionTrackingMode.Rear;
  //public static boolean isWristFlippable = false;

  public static CameraServer camera;


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

  public static boolean getIsAimAssistEnabled() {
    return isAimAssistEnabled;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void bindPilotControls() {
    pilotController.ps.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro(), driveSubsystem));

    // pilotController.touchpad
    //   .onTrue(Commands.either(
    //     Commands.runOnce(() -> driveSubsystem.setHighGear(), driveSubsystem),
    //     Commands.runOnce(() -> driveSubsystem.setLowGear(), driveSubsystem),
    //     () -> driveSubsystem.getGear() == Gear.Low));

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

    testController.cross.onTrue(new InstantCommand(() -> RobotContainer.isAimAssistEnabled = !RobotContainer.isAimAssistEnabled));

    testController.square
      .whileTrue(IntakeCommands.extend())
      .onFalse(IntakeCommands.stop());

    testController.circle
      .whileTrue(IntakeCommands.retract())
      .onFalse(IntakeCommands.stop());

    testController.up
      .onTrue(ClimberCommands.extend())
      .onFalse(ClimberCommands.stop());

    testController.down
      .onTrue(ClimberCommands.retract())
      .onFalse(ClimberCommands.stop());
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
    robotContainerTab.addBoolean("Aim Assist", () -> RobotContainer.isAimAssistEnabled);
  }
}
