package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.controls.DualShock4Controller;
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
import frc.robot.subsystems.shooter.commands.ShootCommand;
import frc.robot.subsystems.shooterFeed.ShooterFeedCommands;
import frc.robot.subsystems.shooterFeed.ShooterFeedSubsystem;
import frc.robot.subsystems.spindexer.SpindexerCommands;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretCommands;
import frc.robot.subsystems.turret.TurretConfig;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class RobotContainer {

  PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);

  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static VisionSubsystem turretVisionSubsystem = new VisionSubsystem("limelight-turret");
  // public static VisionSubsystem shooterVisionSubsystem = new VisionSubsystem("limelight-turret");
  // public static VisionSubsystem shooterVisionSubsystem = new VisionSubsystem("limelight-turret");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  // public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static DeploySubsystem deploySubsystem = new DeploySubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> turretVisionSubsystem.getFeetFromTarget());
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

    turretSubsystem.setDefaultCommand(
      new RunCommand(
        () -> turretSubsystem.setManualOutput(copilotController.getLeftXAxis()),
        turretSubsystem
        )
    );

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
      .onTrue(Commands.sequence(
        new InstantCommand(() -> disableAimAssist()),
        TurretCommands.moveToSafeState(),
        DeployCommands.fullRetract()
      ));

    pilotController.square
      .onTrue(DeployCommands.fullExtend());

    pilotController.cross
      .whileTrue(DeployCommands.agitate())
      .onFalse(DeployCommands.setToHome());

    pilotController.circle
      .onTrue(DeployCommands.setToHome());

    pilotController.l1
      .whileTrue(IntakeCommands.intake())
      .onFalse(IntakeCommands.stop());
      // .whileTrue(Commands.parallel(
      //   DeployCommands.fullExtend(),
      //   IntakeCommands.intake()
      // ));

    pilotController.l2
      .onTrue(ShooterCommands.idleShooter());

    pilotController.r1
      .whileTrue(IntakeCommands.outtake())
      .onFalse(IntakeCommands.stop());

    pilotController.r2
      .whileTrue(new ShootCommand(ShooterConfig.ShooterPreset.PASS));
      // .whileTrue(Commands.parallel(
      //   SpindexerCommands.FeedTowardsFeed(),
      //   ShooterFeedCommands.FeedTowardsShooter()))
      // .onFalse(Commands.parallel(
      //   SpindexerCommands.stop(),
      //   ShooterFeedCommands.stop()));

    // pilotController.left
    //   .whileTrue(TurretCommands.RotateTurretCounterClockwise())
    //   .onFalse(TurretCommands.stop());

    // pilotController.right
    //   .whileTrue(TurretCommands.RotateTurretClockwise())
    //   .onFalse(TurretCommands.stop());

    pilotController.share
      .onTrue(ShooterCommands.idleShooter());

    pilotController.options
      .onTrue(new InstantCommand(() -> toggleAimAssist()));

    pilotController.ps.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro(), driveSubsystem));

     pilotController.touchpad
      .onTrue(Commands.either(
        Commands.runOnce(() -> driveSubsystem.setHighGear(), driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.setLowGear(), driveSubsystem),
        () -> driveSubsystem.getGear() == Gear.Low));

  }

  private void bindCopilotControls() {

    copilotController.triangle
      .onTrue(TurretCommands.moveTo90());

    // copilotController.square
    //   .onTrue();

    copilotController.cross
      .onTrue(ShooterCommands.stop());

    // copilotController.circle
    //   .onTrue(ShooterCommands.idleShooter());

    copilotController.l1
      .whileTrue(SpindexerCommands.FeedAwayFromFeed())
      .onFalse(SpindexerCommands.stop());

    copilotController.l2
      .whileTrue(ShooterFeedCommands.FeedAwayFromShooter())
      .onFalse(ShooterFeedCommands.stop());

    copilotController.r1
      .onTrue(new InstantCommand(() -> toggleAimAssist()));

    copilotController.r2
      .whileTrue(new ShootCommand(ShooterConfig.ShooterPreset.PASS));

    // copilotController.up
    //   .whileTrue(ClimberCommands.extend());

    // copilotController.down
    //   .whileTrue(ClimberCommands.retract());

    // copilotController.left
    //   .whileTrue(TurretCommands.RotateTurretCounterClockwise())
    //   .onFalse(TurretCommands.stop());

    // copilotController.right
    //   .whileTrue(TurretCommands.RotateTurretClockwise())
    //   .onFalse(TurretCommands.stop());

    // copilotController.share
    //   .onTrue();

    // copilotController.options
    //   .onTrue(new InstantCommand(() -> toggleAimAssist()));

    // copilotController.ps
    //   .onTrue();

    //  copilotController.touchpad
    //   .onTrue();

  }

  private void bindTestControls() {

    // testController.left
    //   .whileTrue(TurretCommands.RotateTurretCounterClockwise())
    //   .onFalse(TurretCommands.stop());

    // testController.right
    //   .whileTrue(TurretCommands.RotateTurretClockwise())
    //   .onFalse(TurretCommands.stop());

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
      .onTrue(new InstantCommand(() -> shooterSubsystem.bumpDownRPM()));
      // .whileTrue(DeployCommands.retract())
      // .onFalse(DeployCommands.stop());

    testController.r1
      .onTrue(new InstantCommand(() -> shooterSubsystem.bumpUpRPM()));
      // .whileTrue(DeployCommands.extend())
      // .onFalse(DeployCommands.stop());

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
      .onTrue(TurretCommands.moveTo90());

      // testController.share
    //   .onTrue(ShooterCommands.stop());
  }

  public static void toggleAimAssist() {
    isAimAssistEnabled = !isAimAssistEnabled;
    // If we just enabled it, we MUST tell the turret to stop manual mode
    if (isAimAssistEnabled) {
        // You'll need a way to reference the subsystem here, or
        // better yet, move this logic into a Command.
        turretSubsystem.isManualControl = false;
    }
  }

  public static void disableAimAssist() {
    isAimAssistEnabled = false;
  }

  public static void enableAimAssist() {
    isAimAssistEnabled = true;
  }

  public static SendableChooser<Command> initAutoChooser() {

    NamedCommands.registerCommand("Deploy", DeployCommands.fullExtend());
    NamedCommands.registerCommand("Agitate", DeployCommands.agitatePosition());
    NamedCommands.registerCommand("Intake", IntakeCommands.intake());
    NamedCommands.registerCommand("Intake Off", IntakeCommands.stop().withTimeout(0.25));
    NamedCommands.registerCommand("Stow Intake", DeployCommands.setToHome());
    NamedCommands.registerCommand("Turret to 90", TurretCommands.moveTo90());
    NamedCommands.registerCommand("Aim Assist", new InstantCommand(() -> enableAimAssist()));
    NamedCommands.registerCommand("Shoot", new ShootCommand(ShooterConfig.ShooterPreset.MEDIUM));
    // NamedCommands.registerCommand("Shoot", ShooterCommands.highRPM());
    NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idleShooter());
    NamedCommands.registerCommand("Shooter Off", ShooterCommands.stop());
    NamedCommands.registerCommand("Spindexer Off", SpindexerCommands.stop());
    NamedCommands.registerCommand("Shooter Feed Off", ShooterFeedCommands.stop());
    NamedCommands.registerCommand("Stop Everything", Commands.parallel(
      new InstantCommand(() -> shooterSubsystem.setOff()),
      new InstantCommand(() -> shooterFeedSubsystem.stop()),
      new InstantCommand(() -> spindexerSubsystem.stop()),
      new InstantCommand(() -> intakeSubsystem.stop())
    ));
    // NamedCommands.registerCommand("Move Turret to Center", TurretCommands.moveToCenter());

    var autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    return autoChooser;
  }

  public void initDashboard() {
    var robotContainerTab = Shuffleboard.getTab("Robot Container");

    robotContainerTab.addBoolean("Aim Assist", () -> RobotContainer.isAimAssistEnabled);

    var currentTab = Shuffleboard.getTab("Motor Currents");

    currentTab.addDouble("RF Drive: ", () -> m_pdh.getCurrent(0));
    currentTab.addDouble("RF Steer: ", () -> m_pdh.getCurrent(1));
    currentTab.addDouble("Aux Power: ", () -> m_pdh.getCurrent(2));
    //currentTab.addDouble("Spare: ", () -> m_pdh.getCurrent(3));
    //currentTab.addDouble("Servo Hub: ", () -> m_pdh.getCurrent(4));
    //currentTab.addDouble("Spare: ", () -> m_pdh.getCurrent(5));
    currentTab.addDouble("Intake: ", () -> m_pdh.getCurrent(6));
    currentTab.addDouble("Intake Deploy: ", () -> m_pdh.getCurrent(7));
    currentTab.addDouble("LF Drive: ", () -> m_pdh.getCurrent(8));
    currentTab.addDouble("LF Steer: ", () -> m_pdh.getCurrent(9));
    currentTab.addDouble("LR Drive: ", () -> m_pdh.getCurrent(10));
    currentTab.addDouble("LR Steer: ", () -> m_pdh.getCurrent(11));
    currentTab.addDouble("Turret: ", () -> m_pdh.getCurrent(12));
    currentTab.addDouble("Spindexer: ", () -> m_pdh.getCurrent(13));
    currentTab.addDouble("Shooter Feed: ", () -> m_pdh.getCurrent(14));
    currentTab.addDouble("R Shooter: ", () -> m_pdh.getCurrent(15));
    currentTab.addDouble("L Shooter: ", () -> m_pdh.getCurrent(16));
    //currentTab.addDouble("Climber: ", () -> m_pdh.getCurrent(17));
    currentTab.addDouble("RR Drive: ", () -> m_pdh.getCurrent(18));
    currentTab.addDouble("RR Steer: ", () -> m_pdh.getCurrent(19));
    currentTab.addDouble("RoboRIO 2: ", () -> m_pdh.getCurrent(20));
    currentTab.addDouble("Radio: ", () -> m_pdh.getCurrent(21));
    currentTab.addDouble("Blinkin: ", () -> m_pdh.getCurrent(22));
    //currentTab.addDouble("Spare: ", m_pdh.getCurrent(23));

  // Display total current
    currentTab.addDouble("Total Current", () -> m_pdh.getTotalCurrent());

  }

}
