// package frc.robot.subsystems.climber;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.components.motors.Minion;
// import frc.robot.components.motors.lib.Motor;
// import frc.robot.util.EliteMath;

// import static frc.robot.subsystems.climber.ClimberConfig.*;

// public class ClimberSubsystem extends SubsystemBase {

//     final Motor motor;
//     final PIDController pidController;

//     double targetHeight;
//     double currentHeight;
//     double inchesPerRotation;

//     public ClimberSubsystem() {

//         var config = getMotorConfig();

//         motor = new Minion(config);
//         pidController= new PIDController(config.pidParameters.P, config.pidParameters.I, config.pidParameters.D);

//         targetHeight = config.initialPosition;
//         currentHeight = config.initialPosition;

//         inchesPerRotation = (36.0 * 5.0) / 25.4;

//         initDashboard();
//     }

//     public void periodic() {
//         currentHeight = getPositionInches();
//         double speed = pidController.calculate(currentHeight, targetHeight);
//         speed = EliteMath.clamp(speed, RETRACT_SPEED, EXTEND_SPEED);

//         if (currentHeight <= HOME_POSITION + BUFFER_ZONE && speed < 0) {
//             double distanceIntoBuffer = currentHeight - HOME_POSITION;
//             double scale = EliteMath.map(distanceIntoBuffer, 0, BUFFER_ZONE, 0, 1);
//             speed *= Math.max(0, scale);  // scale speed down to zero at bottom
//         }

//         if (currentHeight <= HOME_POSITION && speed < 0) {
//             speed = 0;
//             targetHeight = Math.max(targetHeight, HOME_POSITION);
//         }

//         if (currentHeight >= EXTEND_LIMIT && speed > 0) {
//             speed = 0;
//             targetHeight = Math.min(targetHeight, EXTEND_LIMIT);
//         }

//         motor.setSpeed(speed);
//     }

//     public void extend() {
//         targetHeight = Math.min(targetHeight + MANUAL_MODIFIER, EXTEND_LIMIT);
//     }

//     public void retract() {
//         targetHeight = Math.max(targetHeight - MANUAL_MODIFIER, HOME_POSITION);
//     }

//     public void setHeight(double inches) {
//         targetHeight = inches;
//     }

//     public boolean isAtTargetHeight() {
//         return Math.abs(targetHeight - currentHeight) <= TARGET_THRESHOLD;
//     }

//     public double getPosition() {
//         // Pulley motor rotations
//         return motor.getPosition();
//     }

//     public double getPositionInches() {
//         // Inches per pulley rotation
//         return motor.getPosition() * inchesPerRotation;
//     }

//     public double getTargetHeightInches() {
//         return targetHeight;
//     }

//     public void stop() {
//         motor.stop();
//         targetHeight = currentHeight;
//     }

//     public void initDashboard() {
//         var tab = Shuffleboard.getTab("Climber");

//         tab.addDouble("Position", () -> getPosition())
//         .withPosition(0, 0)
//         .withSize(2, 1);

//         tab.addDouble("Position Inches", () -> getPositionInches())
//         .withPosition(2, 0)
//         .withSize(2, 1);

//         tab.addDouble("Target Height", () -> targetHeight)
//             .withPosition(4, 0)
//             .withSize(2, 1);
//     }
// }
