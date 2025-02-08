package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    static class ArmIOInputs {
        public double shoulderPositionRad = 0.0;
        public double wristPositionRad = 0.0;
        public double shoulderVelocityRadPerSec = 0.0;
        public double wristVelocityRadPerSec = 0.0;
        public double shoulderAppliedVolts = 0.0;
        public double wristAppliedVolts = 0.0;
        public double[] shoulderSupplyCurrentAmps = new double[] {};
        public double[] wristSupplyCurrentAmps = new double[] {};
        public double[] shoulderTorqueCurrentAmps = new double[] {};
        public double[] wristTorqueCurrentAmps = new double[] {};
        public double[] shouldertTempCelsius = new double[] {};
        public double[] wristTempCelsius = new double[] {};
    }

    default void updateInputs(ArmIOInputs inputs) {}
    public default void setShoulderVoltage(double volts) {}
    public default void setWristVoltage(double volts) {}
    public default void setBrakeMode(boolean shoulderBrake, boolean wristBrake) {}
    
    
    
}
