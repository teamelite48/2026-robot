package frc.robot.components.motors.lib;

import frc.robot.lib.PIDParameters;

public class MotorConfig {
    public String canivoreBus;
    public int canBusId;
    public boolean isInverted = false;
    public boolean isBrakeModeEnabled = false;
    public double positionConversionFactor = 1.0;
    public double feedForward = 0.0;
    public Integer currentLimit = null;
    public Double forwardLimit = null;
    public Double reverseLimit = null;
    public Double initialPosition = null;
    public Double maxForwardSpeed = null;
    public Double maxReverseSpeed = null;
    public Double velocityConversionFactor = null;
    public PIDParameters pidParameters = null;

    public MotorConfig(int canBusId) {
        this.canivoreBus = null;
        this.canBusId = canBusId;
    }

    public MotorConfig(int canBusId, String canivore) {
        this.canivoreBus = canivore;
        this.canBusId = canBusId;
    }
}
