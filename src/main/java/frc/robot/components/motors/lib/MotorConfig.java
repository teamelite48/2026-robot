package frc.robot.components.motors.lib;

import com.ctre.phoenix6.CANBus;
import frc.robot.lib.PIDParameters;

public class MotorConfig {
    public CANBus canBus = null;  // Used for canivore
    public int canBusId;
    public boolean isInverted = false;
    public boolean isBrakeModeEnabled = false;
    public boolean enableFOC = false;  // Can only enable if we have Phoenix Pro License
    public boolean continuousWrap = false;
    public double positionConversionFactor = 1.0;
    public double feedForwardVolts = 0.0;
    
    public Integer currentLimit = null;
    
    public Integer supplyCurrentLimit = null;  
    public Integer supplyCurrentLowerLimit = null;
    public Integer supplyCurrentLowerTime = null;
    public Integer statorCurrentLimit = null;
    
    public Double forwardLimit = null;
    public Double reverseLimit = null;
    public Double initialPosition = null;
    public Double maxForwardSpeed = null;
    public Double maxReverseSpeed = null;
    public Double velocityConversionFactor = null;
    public Double motionMagicCruiseVelocity = null;
    public Double motionMagicAcceleration = null;
    public Double motionMagicJerk = null;
    public PIDParameters pidParameters = null;

    public MotorConfig(int canBusId) {
        this.canBusId = canBusId;
    }

    public MotorConfig(int canBusId, CANBus bus) {
        this.canBus = bus;
        this.canBusId = canBusId;
    }
}
