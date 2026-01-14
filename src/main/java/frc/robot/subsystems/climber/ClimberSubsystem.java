package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.motors.Kraken;
import frc.robot.components.motors.lib.Motor;
import frc.robot.util.EliteMath;

import static frc.robot.subsystems.climber.ClimberConfig.*;

public class ClimberSubsystem extends SubsystemBase {

    final Motor motor;
    final PIDController pidController;

    public ClimberSubsystem() {

        var config = getMotorConfig();

        motor = new Kraken(config);
        pidController= new PIDController(config.pidParameters.P, config.pidParameters.I, config.pidParameters.D);

        initDashboard();
    }

    public void periodic() {}

    public void extend() {

        if (getPosition() >= ClimberConfig.EXTEND_LIMIT) {
            stop();
        }
        else {
            motor.setSpeed(EXTEND_SPEED);
        }
    }

    public void retract() {

        if (getPosition() <= ClimberConfig.RETRACT_LIMIT) {
            stop();
        }
        else {
            motor.setSpeed(RETRACT_SPEED);
        }
    }

    public double getPosition() {
        return motor.getPosition();
    }

    public void stop() {
        motor.stop();
    }

    public void initDashboard() {
        var tab = Shuffleboard.getTab("Climber");

        tab.addDouble("Position", () -> getPosition())
        .withPosition(0, 0)
        .withSize(2, 1);
    }
}
