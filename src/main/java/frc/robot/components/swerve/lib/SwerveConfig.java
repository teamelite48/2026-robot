package frc.robot.components.swerve.lib;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.lib.PIDParameters;


public final class SwerveConfig {

    private final double wheelDiameterMeters;
    private final double trackWidthMeters;
    private final double wheelbaseMeters;
    private final double maxMetersPerSecond;
    private final double driveReduction;
    private final double angleReduction;

    public final int driveMotorSupplyCurrentLimit;
    public final int driveMotorSupplyCurrentLowerLimit;
    public final int driveMotorSupplyTimeThreshold;
    public final int driveMotorStatorCurrentLimit;
    public final int angleMotorSupplyCurrentLimit;
    public final int angleMotorSupplyCurrentLowerLimit;
    public final int angleMotorSupplyTimeThreshold;
    public final int angleMotorStatorCurrentLimit;

    public final double slewRate;
    public final double maxGearSpeed;
    public final double lowGearSpeed;
    public final double strafeSpeed;
    public final double nominalVoltage;
    public final boolean isAngleMotorInverted;
    public final boolean isInverted;

    private final PIDParameters movingPid;
    private final PIDParameters standingPid;

    private SwerveConfig(Builder b) {
        wheelDiameterMeters = b.wheelDiameterMeters;
        trackWidthMeters = b.trackWidthMeters;
        wheelbaseMeters = b.wheelbaseMeters;
        maxMetersPerSecond = b.maxMetersPerSecond;
        driveReduction = b.driveReduction;
        angleReduction = b.angleReduction;
        movingPid = b.movingPid;
        standingPid = b.standingPid;

        driveMotorSupplyCurrentLimit = b.driveMotorSupplyCurrentLimit;
        driveMotorSupplyCurrentLowerLimit = b.driveMotorSupplyCurrentLowerLimit;
        driveMotorSupplyTimeThreshold = b.driveMotorSupplyTimeThreshold;
        driveMotorStatorCurrentLimit = b.driveMotorStatorCurrentLimit;

        angleMotorSupplyCurrentLimit = b.angleMotorSupplyCurrentLimit;
        angleMotorSupplyCurrentLowerLimit = b.angleMotorSupplyCurrentLowerLimit;
        angleMotorSupplyTimeThreshold = b.angleMotorSupplyTimeThreshold;
        angleMotorStatorCurrentLimit = b.angleMotorStatorCurrentLimit;

        slewRate = b.slewRate;
        maxGearSpeed = b.maxGearSpeed;
        lowGearSpeed = b.lowGearSpeed;
        strafeSpeed = b.strafeSpeed;
        nominalVoltage = b.nominalVoltage;
        isAngleMotorInverted = b.isAngleMotorInverted;
        isInverted = b.isInverted;
    }

    public double getMaxAngularMetersPerSecond() {
        return maxMetersPerSecond / Math.hypot(trackWidthMeters / 2.0, wheelbaseMeters / 2.0);
    }

    public double driveMetersPerRotation() {
        return Math.PI * wheelDiameterMeters * driveReduction;
    }

    public double angleRadiansPerRotation() {
        return 2.0 * Math.PI * angleReduction;
    }

    public double maxAngularSpeed() {
        double radius = Math.hypot(trackWidthMeters / 2.0, wheelbaseMeters / 2.0);
        return maxMetersPerSecond / radius;
    }

    public double getLowGearSpeed() {
        return this.lowGearSpeed;
    }

    public double getMaxGearSpeed() {
        return this.maxGearSpeed;
    }

    public double getMaxMetersPerSecond() {
        return this.maxMetersPerSecond;
    }

    public PIDParameters getMovingPid() {
        return this.movingPid;
    }

    public PIDParameters getStandingPid() {
        return this.standingPid;
    }

    public int getDriveMotorSupplyCurrentLimit() {
        return this.driveMotorSupplyCurrentLimit;
    }

    public int getDriveMotorSupplyCurrentLowerLimit() {
        return this.driveMotorSupplyCurrentLimit;
    }

    public int getDriveMotorSupplyCurrentTimeThreshold() {
        return this.driveMotorSupplyTimeThreshold;
    }

    public int getDriveMotorStatorCurrentLimit() {
        return this.driveMotorStatorCurrentLimit;
    }

    public int getAngleMotorSupplyCurrentLimit() {
        return this.angleMotorSupplyCurrentLimit;
    }

    public int getAngleMotorSupplyCurrentLowerLimit() {
        return this.angleMotorSupplyCurrentLimit;
    }

    public int getAngleMotorSupplyCurrentTimeThreshold() {
        return this.angleMotorSupplyTimeThreshold;
    }

    public int getAngleMotorStatorCurrentLimit() {
        return this.angleMotorStatorCurrentLimit;
    }

    public double getTrackWidthMeters() {
        return this.trackWidthMeters;
    }

    public double getWheelBaseMeters() {
        return this.wheelbaseMeters;
    }



    public static class Builder {

        private double wheelDiameterMeters = 0.1016;
        private double trackWidthMeters = 0.5;
        private double wheelbaseMeters = 0.5;
        private double maxMetersPerSecond = 4.5;

        public int driveMotorSupplyCurrentLowerLimit = 50;      // Amps
        public int driveMotorSupplyCurrentLimit = 70;           // Allow a brief spike up to this current
        public int driveMotorSupplyTimeThreshold = 3;           // Seconds before dropping to Lower Limit
        public int driveMotorStatorCurrentLimit = 150;

        public int angleMotorSupplyCurrentLowerLimit = 30;      // Amps
        public int angleMotorSupplyCurrentLimit = 45;           // Allow a brief spike up to this current
        public int angleMotorSupplyTimeThreshold = 1;           // Seconds before dropping to Lower Limit
        public int angleMotorStatorCurrentLimit = 90;

        public double slewRate = 2.0;
        public double maxGearSpeed = 1.0;
        public double lowGearSpeed = 0.55;
        public double strafeSpeed = 0.1;
        public double nominalVoltage = 12.0;
        public boolean isAngleMotorInverted = false;
        public boolean isInverted = true;

        private double driveReduction = 1.0 / 6.75;
        private double angleReduction = 1.0 / 21.4;

        private PIDParameters movingPid = new PIDParameters(0.005, 0, 0, 0, 0);
        private PIDParameters standingPid = new PIDParameters(0.01, 0, 0, 0, 0);

        public Builder wheelDiameter(double v) {
            wheelDiameterMeters = v;
            return this;
        }

        public Builder setTrackWidth(double v) {
            trackWidthMeters = v;
            return this;
        }

        public Builder wheelbase(double v) {
            wheelbaseMeters = v;
            return this;
        }

        public Builder maxSpeed(double v) {
            maxMetersPerSecond = v;
            return this;
        }

        public Builder driveReduction(double v) {
            driveReduction = v;
            return this;
        }

        public Builder angleReduction(double v) {
            angleReduction = v;
            return this;
        }

        public Builder setMovingPid(PIDParameters p) {
            movingPid = p;
            return this;
        }

        public Builder setStandingPid(PIDParameters p) {
            standingPid = p;
            return this;
        }

        public Builder setDriveMotorSupplyCurrentLimit(int l) {
            driveMotorSupplyCurrentLimit = l;
            return this;
        }

        public Builder setAngleMotorCurrentLimit(int l) {
            angleMotorSupplyCurrentLimit = l;
            return this;
        }

        public Builder setAngleInversion(boolean f) {
            isAngleMotorInverted = f;
            return this;
        }

        public SwerveConfig build() {
            return new SwerveConfig(this);
        }
    }
}


// public class SwerveConfig {
//     public double maxGearSpeed = 1.0;
//     public double lowGearSpeed = maxGearSpeed / 2.0;
//     public double strafeSpeed = 0.1;
//     public double slewRate = 2.0;
//     public double wheelDiameterMeters;
//     public double trackWidthMeters;
//     public double wheelbaseMeters;
//     public double maxMetersPerSecond;
//     public double maxAngularMetersPerSecond = maxMetersPerSecond / Math.hypot(trackWidthMeters / 2.0, wheelbaseMeters / 2.0);
//     public double nominalVoltage = 12.0;
//     public int driveMotorCurrentLimit = 80;
//     public int angleMotorCurrentLimit = 80;
//     public SensorDirectionValue angleMotorAbsoluteEncoderInversion = SensorDirectionValue.CounterClockwise_Positive;
//     // public boolean isInverted = true;
//     public boolean isAngleMotorInverted = false;
//     public double driveMotorReduction = (1.0 / 6.0);
//     public double angleMotorReduction = (1.0 / 26.0);
//     public double drivePositionToMetersConversionFactor = Math.PI * wheelDiameterMeters * driveMotorReduction;
//     public double anglePositionToRadiansConversionFactor = 2.0 * Math.PI * angleMotorReduction;
//     public int encoderResetIterations = 500;
//     public double encoderResetMaxAngularVelocity = Math.toRadians(0.5);
//     public double rotationBreakawayOutput = 0.0;  //0.05;
//     public PIDParameters movingRotationPid = null;
//     public PIDParameters standingRotationPid = null;
// }
