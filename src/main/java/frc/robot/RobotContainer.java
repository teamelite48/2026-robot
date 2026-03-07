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
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.controls.DualShock4Controller;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deploy.DeployCommands;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.Gear;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterFeed.ShooterFeedCommands;
import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
import frc.robot.subsystems.spindexer.SpindexerCommands;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretCommands;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionCommands;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  // public static VisionSubsystem collectorVisionSubsystem = new VisionSubsystem("limelight-collector");
  public static VisionSubsystem shooterVisionSubsystem = new VisionSubsystem("limelight-shooter");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static DeploySubsystem deploySubsystem = new DeploySubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> shooterVisionSubsystem.getFeetFromTarget());
  public static ShooterFeedSubsystem shooterFeedSubsystem = new ShooterFeedSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();

  private final SendableChooser<Command> autoChooser;

  public static boolean isAimAssistEnabled = true;
  public static boolean isShooting = false;
  public static boolean isAutonomous = false;
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

    pilotController.triangle
      .whileTrue(Commands.sequence(
        // VisionCommands.turnOffVision(),
        TurretCommands.moveToSafeState(),
        DeployCommands.fullRetract(),
        ClimberCommands.extend()
      ));

    pilotController.square
      .onTrue(DeployCommands.extend());

    pilotController.cross
      .whileTrue(ClimberCommands.retract());

    pilotController.circle
      .onTrue(DeployCommands.setToHome());

    pilotController.l1
      .whileTrue(IntakeCommands.intake());

    pilotController.l2
      .onTrue(ShooterCommands.idleShooter());

    pilotController.r1
      .whileTrue(IntakeCommands.outtake());

    pilotController.r2
      .whileTrue(Commands.parallel(
        SpindexerCommands.FeedTowardsFeed(),
        ShooterFeedCommands.FeedTowardsShooter()))
      .onFalse(Commands.parallel(
        SpindexerCommands.stop(),
        ShooterFeedCommands.stop()));

    pilotController.left
      .whileTrue(DriveCommands.strafeLeft());

    pilotController.right
      .whileTrue(DriveCommands.strafeRight());

    pilotController.share
      .onTrue(ShooterCommands.idleShooter());

    // pilotController.options
    //   .onTrue(toggleAutoAim());

    pilotController.ps.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro(), driveSubsystem));

     pilotController.touchpad
      .onTrue(Commands.either(
        Commands.runOnce(() -> driveSubsystem.setHighGear(), driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.setLowGear(), driveSubsystem),
        () -> driveSubsystem.getGear() == Gear.Low));

  }

  private void bindCopilotControls() {

    // copilotController.triangle
    //   .whileTrue();

    // copilotController.square
    //   .onTrue();

    // copilotController.cross
    //   .whileTrue();

    copilotController.circle
      .onTrue(ShooterCommands.idleShooter());

    copilotController.l1
      .whileTrue(new ShootCommand());

    // copilotController.l2
    //   .whileTrue();

    copilotController.r1
      .whileTrue(SpindexerCommands.FeedTowardsFeed());

    // copilotController.r2
    //   .whileTrue();

    copilotController.up
      .whileTrue(ClimberCommands.extend());

    copilotController.down
      .whileTrue(ClimberCommands.retract());

    // copilotController.left
    //   .whileTrue();

    // copilotController.right
    //   .whileTrue();

    // copilotController.share
    //   .onTrue();

    // copilotController.options
    //   .onTrue(toggleAutoAim());

    // copilotController.ps
    //   .onTrue();

    //  copilotController.touchpad
    //   .onTrue();

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
      .onTrue(Commands.parallel(
        DeployCommands.extend(),
        IntakeCommands.intake()))
      .onFalse(DeployCommands.stop());

    testController.circle
      .onTrue(Commands.parallel(
        DeployCommands.setToHome(),
        IntakeCommands.intake()))
      .onFalse(Commands.parallel(
        DeployCommands.stop(),
        IntakeCommands.stop()));

    // Bounces behavior (angry PID)
    testController.triangle
      .onTrue(DeployCommands.setToHome())
      // .onFalse(Commands.parallel(
      //   DeployCommands.stop(),
      //   IntakeCommands.stop()))
      ;

    testController.up
      .whileTrue(ClimberCommands.extend())
      .onFalse(ClimberCommands.stop());

    testController.down
      .whileTrue(ClimberCommands.retract())
      .onFalse(ClimberCommands.stop());

      testController.share
      .onTrue(ShooterCommands.stop());
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
