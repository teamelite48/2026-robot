package frc.robot.components.swerve.lib;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.lib.PIDParameters;

public class SwerveConfig {
    public double maxGearSpeed = 1.0;
    public double lowGearSpeed = maxGearSpeed / 2.0;
    public double strafeSpeed = 0.1;
    public double slewRate = 2.0;
    public Double wheelDiameter = null;
    public Double trackWidthMeters = null;
    public Double wheelbaseMeters = null;
    public Double maxMetersPerSecond = null;
    public double maxAngularMetersPerSecond = maxMetersPerSecond / Math.hypot(trackWidthMeters / 2.0, wheelbaseMeters / 2.0);
    public double nominalVoltage = 12.0;
    public int driveMotorCurrentLimit = 55;
    public int angleMotorCurrentLimit = 30;
    public SensorDirectionValue angleMotorAbsolueEncoderInversion = SensorDirectionValue.CounterClockwise_Positive;
    // public boolean isInverted = true;
    public boolean isAngleMotorInverted = false;
    public double driveMotorReduction = (1.0 / 6.0);
    public double angleMotorReduction = (1.0 / 25.0);
    public double drivePositionToMetersConversionFactor = Math.PI * wheelDiameter * driveMotorReduction;
    public double anglePositionToRadiansConversionFactor = 2.0 * Math.PI * angleMotorReduction;
    public int encoderResetIterations = 500;
    public double encoderResetMaxAngularVelocity = Math.toRadians(0.5);
    public double rotationBreakawayOutput = 0.0;  //0.05;
    public PIDParameters movingRotationPid = null;
    public PIDParameters standingRotationPid = null;
}
