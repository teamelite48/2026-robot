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
import frc.robot.commands.Arm.ArmCommands;
import frc.robot.commands.Robot.RobotCommands;
import frc.robot.controls.DualShock4Controller;
import frc.robot.subsystems.claw.ClawCommands;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.Gear;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.shoulder.ShoulderCommands;
import frc.robot.subsystems.shoulder.ShoulderSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristCommands;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RobotContainer {

  DualShock4Controller pilotController = new DualShock4Controller(0);
  DualShock4Controller copilotController = new DualShock4Controller(1);
  DualShock4Controller testController = new DualShock4Controller(2);



  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static VisionSubsystem rearVisionSubsystem = new VisionSubsystem("limelight-rear");
  public static VisionSubsystem frontVisionSubsystem = new VisionSubsystem("limelight-front");
  public static LedSubsystem ledSubsystem = new LedSubsystem();
  public static WristSubsystem wristSubsystem = new WristSubsystem();
  public static ClimberSubsystem climberSubsytem = new ClimberSubsystem();
  public static ClawSubsystem clawSubsystem = new ClawSubsystem();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static ShoulderSubsystem shoulderSubsystem = new ShoulderSubsystem();
  private final SendableChooser<Command> autoChooser;

  public static GamePiece gamePieceMode = GamePiece.Coral;
  public static VisionTrackingMode visionTrackingMode = VisionTrackingMode.Rear;
  public static boolean isWristFlippable = false;

  public static CameraServer camera;

  public RobotContainer() {

    autoChooser = RobotContainer.initAutoChooser();

    CameraServer.startAutomaticCapture();

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

    pilotController.touchpad
      .onTrue(Commands.either(
        Commands.runOnce(() -> driveSubsystem.setHighGear(), driveSubsystem),
        Commands.runOnce(() -> driveSubsystem.setLowGear(), driveSubsystem),
        () -> driveSubsystem.getGear() == Gear.Low));

    pilotController.left
      .whileTrue(DriveCommands.strafeRight());

    pilotController.right
      .whileTrue(DriveCommands.strafeLeft());

    pilotController.circle
      .onTrue(ArmCommands.HomeArm());

    pilotController.options
      .onTrue(RobotCommands.ToggleGamePieceMode());

    pilotController.square
      .onTrue(ArmCommands.LowAlgaeAndStationPickup());

    pilotController.cross
      .onTrue(ArmCommands.FloorPickup());

    pilotController.l2
      .whileTrue(ClawCommands.intake());

    pilotController.r2
      .whileTrue(ClawCommands.outtake());

    pilotController.triangle
      .onTrue(ArmCommands.L3Pickup());
  }

  private void bindCopilotControls() {

    copilotController.touchpad
      .onTrue(ArmCommands.HomeArm());

    copilotController.ps
      .onTrue(RobotCommands.ToggleGamePieceMode());

    copilotController.l1
      .whileTrue(ClawCommands.intake());

    copilotController.r1
      .whileTrue(ClawCommands.outtake());

    copilotController.l2
      .whileTrue(ClimberCommands.retract())
      .onFalse(ClimberCommands.stop());

    copilotController.r2
      .whileTrue(ClimberCommands.extend())
      .onFalse(ClimberCommands.stop());

    copilotController.left
      .onTrue(ArmCommands.HangingPosition());

    copilotController.right
      .onTrue(WristCommands.flip());

    copilotController.cross
      .onTrue(ArmCommands.L1andProcessorScore());

    copilotController.triangle
      .onTrue(ArmCommands.L4andBargeScore());

    copilotController.circle
      .onTrue(ArmCommands.L3Score());

    copilotController.square
      .onTrue(ArmCommands.L2Score());

    new Trigger(() -> copilotController.getLeftAxes().getY() < 0)
        .whileTrue(ElevatorCommands.extend());

    new Trigger(() -> copilotController.getLeftAxes().getY() > 0)
        .whileTrue(ElevatorCommands.retract());

    new Trigger(() -> copilotController.getRightAxes().getY() > 0)
        .whileTrue(ShoulderCommands.tiltUp());

    new Trigger(() -> copilotController.getRightAxes().getY() < 0)
        .whileTrue(ShoulderCommands.tiltDown());
  }

  private void bindTestControls() {

    testController.l1
        .whileTrue(ClawCommands.intake());

    testController.r1
        .whileTrue(ClawCommands.outtake());

    testController.up
      .whileTrue(ElevatorCommands.extend());

    testController.down
      .whileTrue(ElevatorCommands.retract());

    testController.left
        .whileTrue(ShoulderCommands.tiltUp());

    testController.right
        .whileTrue(ShoulderCommands.tiltDown());

    new Trigger(() -> testController.getLeftAxes().getY() > 0)
        .whileTrue(WristCommands.tiltUp());

    new Trigger(() -> testController.getLeftAxes().getY() < 0)
        .whileTrue(WristCommands.tiltDown());

    new Trigger(() -> testController.getRightAxes().getX() > 0)
        .whileTrue(WristCommands.rotateCounterClockwise());

    new Trigger(() -> testController.getRightAxes().getX() < 0)
        .whileTrue(WristCommands.rotateClockwise());

    // testController.circle.onTrue(WristCommands.goHome());
    // testController.circle.onTrue(ShoulderCommands.setAngle(90.0));
    // testController.circle.onTrue(ElevatorCommands.setHeight(5.0));
    testController.circle.onTrue(ArmCommands.HomeArm());
    testController.ps.onTrue(RobotCommands.ToggleGamePieceMode());

    testController.cross
      .onTrue(ArmCommands.L4andBargeScore());

    testController.touchpad
      .whileTrue(DriveCommands.seekTarget());

    // new Trigger(() -> Math.abs(testController.getRightAxes().getX()) > 0)
    // .whileTrue(testController.getRightAxes().getX() > 0 ?
    // WristCommands.rotateClockwise() : WristCommands.rotateCounterClockwise())
    // .onFalse(WristCommands.stop());

    // testController.circle.onTrue(zeroWrist);
    // testController.cross.onTrue(testWrist1);
    // testController.triangle.onTrue(testWrist2);
  }

  public static SendableChooser<Command> initAutoChooser() {

    NamedCommands.registerCommand("Seek Target", DriveCommands.seekTarget());
    NamedCommands.registerCommand("Home Arm", ArmCommands.HomeArm());
    NamedCommands.registerCommand("Arm to L4", ArmCommands.L4andBargeScore());
    NamedCommands.registerCommand("Outtake", ClawCommands.outtake());
    NamedCommands.registerCommand("Intake", ClawCommands.intake());
    NamedCommands.registerCommand("Stop Intake", ClawCommands.stop());
    NamedCommands.registerCommand("Collect", ArmCommands.LowAlgaeAndStationPickup());
    NamedCommands.registerCommand("Arm to L1", ArmCommands.L1andProcessorScore());
    NamedCommands.registerCommand("Toggle Game Piece Mode", RobotCommands.ToggleGamePieceMode());
    NamedCommands.registerCommand("Hold Intake", ClawCommands.hold());


    var autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    return autoChooser;
  }

  public void initDashboard() {
    var robotContainerTab = Shuffleboard.getTab("Robot Container");

    robotContainerTab.addString("Game Piece Mode", () -> gamePieceMode.toString())
      .withPosition(0, 0)
      .withSize(2, 1);

    robotContainerTab.addString("Vision Tracking Mode", () -> RobotContainer.visionTrackingMode.toString())
      .withPosition(2, 0)
      .withSize(2, 1);

    robotContainerTab.addString("Current Vision Target", () -> {
      return RobotContainer.visionTrackingMode == VisionTrackingMode.Front
        ? frontVisionSubsystem.getTargetName()
        : rearVisionSubsystem.getTargetName();
    })
      .withPosition(2, 1)
      .withSize(2, 1);

    robotContainerTab.addBoolean("Is Wrist Flippable", () -> RobotContainer.isWristFlippable)
      .withPosition(4, 0)
      .withSize(1, 1);
  }
}
