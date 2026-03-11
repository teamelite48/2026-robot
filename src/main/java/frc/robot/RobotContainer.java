package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.subsystems.shooter.ShooterConfig;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterMode;
import frc.robot.subsystems.shooter.commands.ShootCommand;
import frc.robot.subsystems.shooterFeed.ShooterFeedCommands;
import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
import frc.robot.subsystems.spindexer.SpindexerCommands;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretCommands;
import frc.robot.subsystems.turret.TurretConfig;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static VisionSubsystem shooterVisionSubsystem = new VisionSubsystem("limelight-turret");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static DeploySubsystem deploySubsystem = new DeploySubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> shooterVisionSubsystem.getFeetFromTarget());
  public static ShooterFeedSubsystem shooterFeedSubsystem = new ShooterFeedSubsystem();
  public static SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem();

  private final SendableChooser<Command> autoChooser;

  public static boolean isAimAssistEnabled = false;
  public static boolean isShooting = false;
  public static boolean isAutonomous = false;

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
        new InstantCommand(() -> disableAimAssist()),
        TurretCommands.moveToSafeState(),
        DeployCommands.fullRetract(),
        ClimberCommands.extend()
      ));

    pilotController.square
      .onTrue(DeployCommands.fullExtend());

    pilotController.cross
      .whileTrue(ClimberCommands.retract());

    pilotController.circle
      .onTrue(DeployCommands.setToHome());

    pilotController.l1
      .whileTrue(Commands.parallel(
        DeployCommands.fullExtend(),
        IntakeCommands.intake()
      ));

    pilotController.l2
      .onTrue(ShooterCommands.idleShooter());

    pilotController.r1
      .whileTrue(IntakeCommands.outtake());

    pilotController.r2
      .whileTrue(new ShootCommand(ShooterConfig.ShooterPreset.MEDIUM));
      // .whileTrue(Commands.parallel(
      //   SpindexerCommands.FeedTowardsFeed(),
      //   ShooterFeedCommands.FeedTowardsShooter()))
      // .onFalse(Commands.parallel(
      //   SpindexerCommands.stop(),
      //   ShooterFeedCommands.stop()));

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
      .whileTrue(SpindexerCommands.FeedAwayFromFeed());

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

    copilotController.left
      .whileTrue(TurretCommands.RotateTurretCounterClockwise())
      .onFalse(TurretCommands.stop());

    copilotController.right
      .whileTrue(TurretCommands.RotateTurretClockwise())
      .onFalse(TurretCommands.stop());

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

    // testController.square
    //   .onTrue(ShooterCommands.idleShooter());

    // testController.circle
    //   .onTrue(ShooterCommands.stop());

    // testController.cross.onTrue(new InstantCommand(() -> RobotContainer.isAimAssistEnabled = !RobotContainer.isAimAssistEnabled));

    // testController.square
    //   .onTrue(Commands.parallel(
    //     DeployCommands.fullExtend(),
    //     IntakeCommands.intake()))
    //   .onFalse(DeployCommands.stop());

    // testController.circle
    //   .onTrue(Commands.parallel(
    //     DeployCommands.setToHome(),
    //     IntakeCommands.intake()))
    //   .onFalse(Commands.parallel(
    //     DeployCommands.stop(),
    //     IntakeCommands.stop()));

    // testController.triangle
    //   .onTrue(DeployCommands.setToHome())
    //   .onFalse(IntakeCommands.stop());

    testController.square
      .onTrue(ShooterCommands.lowRPM());

    testController.triangle
      .onTrue(ShooterCommands.mediumRPM());

    testController.circle
      .onTrue(ShooterCommands.highRPM());

    testController.cross
      .onTrue(new InstantCommand(() -> shooterSubsystem.setOff()));
      // .onTrue(ShooterCommands.idleShooter());

    testController.l1
      .whileTrue(DeployCommands.retract())
      .onFalse(DeployCommands.stop());

    testController.r1
      .whileTrue(DeployCommands.extend())
      .onFalse(DeployCommands.stop());

    testController.l2
      .whileTrue(SpindexerCommands.FeedAwayFromFeed())
      .onFalse(SpindexerCommands.stop());

    testController.r2
      .whileTrue(SpindexerCommands.FeedTowardsFeed())
      .onFalse(SpindexerCommands.stop());

    testController.up
      .whileTrue(ShooterFeedCommands.FeedTowardsShooter())
      .onFalse(ShooterFeedCommands.stop());

    testController.down
      .whileTrue(ShooterFeedCommands.FeedAwayFromShooter())
      .onFalse(ShooterFeedCommands.stop());

    testController.share
      .whileTrue(IntakeCommands.intake())
      .onFalse(IntakeCommands.stop());

    testController.options
      .whileTrue(IntakeCommands.outtake())
      .onFalse(IntakeCommands.stop());

    testController.touchpad
      .onTrue(new InstantCommand(() -> turretSubsystem.moveToDegrees(TurretConfig.HOME_POSITION), turretSubsystem));

     testController.l3
      .onTrue(new InstantCommand(() -> turretSubsystem.moveToDegrees(TurretConfig.degreesAtCenter), turretSubsystem));

      // testController.share
    //   .onTrue(ShooterCommands.stop());
  }

  public static void toggleAimAssist() {
    isAimAssistEnabled = !isAimAssistEnabled;
  }

  public static void disableAimAssist() {
    isAimAssistEnabled = false;
  }

  public static void enableAimAssist() {
    isAimAssistEnabled = true;
  }

  public static SendableChooser<Command> initAutoChooser() {

    NamedCommands.registerCommand("Deploy", DeployCommands.fullExtend());
    NamedCommands.registerCommand("Intake", IntakeCommands.intake());
    NamedCommands.registerCommand("Home", Commands.parallel(TurretCommands.moveToSafeState(), DeployCommands.setToHome(), IntakeCommands.stop()));
    NamedCommands.registerCommand("Aim Assist", new InstantCommand(() -> enableAimAssist()));
    // NamedCommands.registerCommand("Shoot", new ShootCommand(ShooterConfig.ShooterPreset.HIGH));
    NamedCommands.registerCommand("Shoot", ShooterCommands.highRPM());
    NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idleShooter());
    NamedCommands.registerCommand("Move Turret to Center", TurretCommands.moveToCenter());

    var autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    return autoChooser;
  }

  public void initDashboard() {
    var robotContainerTab = Shuffleboard.getTab("Robot Container");

    robotContainerTab.addBoolean("Aim Assist", () -> RobotContainer.isAimAssistEnabled);
  }
}
