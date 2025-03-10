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
        public double shoulderSupplyCurrentAmps = 0.0;
        public double wristSupplyCurrentAmps = 0.0;
        public double shoulderTempCelsius = 0.0;
        public double wristTempCelsius = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {}
    public default void setVoltage(double volts) {}
    public default void setMotorBrake(boolean brake) {}
    
    
}