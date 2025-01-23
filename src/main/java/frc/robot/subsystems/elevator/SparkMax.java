package frc.robot.subsystems.elevator;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class SparkMax {
    @SuppressWarnings("unused")
    private int channel;
    @SuppressWarnings("unused")
    private MotorType motorType;
    @SuppressWarnings("unused")
    private int currentLimit;
    @SuppressWarnings("unused")
    private boolean inverted;
    @SuppressWarnings("unused")
    private boolean brakeMode;
    @SuppressWarnings("unused")
    private double rampRate;
    private double speed;
    private double position;
    private double current;
    private double temperature;

    public SparkMax(int channel) {
        this.channel = channel;
    }

    public void setMotorType(MotorType motorType) {
        this.motorType = motorType;
    }

    public void setCurrentLimit(int currentLimit) {
        this.currentLimit = currentLimit;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void setBrakeMode(boolean brakeMode) {
        this.brakeMode = brakeMode;
    }

    public void setRampRate(double rampRate) {
        this.rampRate = rampRate;
    }

    public RelativeEncoder getEncoder() {
        return new RelativeEncoder() {
            @Override
            public double getPosition() {
                return position;
            }

            @Override
            public double getVelocity() {
                return speed;
            }

            @Override
            public REVLibError setPosition(double position) {
                SparkMax.this.position = position;
                                return null;
            }
        };
    }

    public void set(double speed) {
        this.speed = speed;
    }

    public double getOutputCurrent() {
        return current;
    }

    public double getMotorTemperature() {
        return temperature;
    }

    // Add other necessary methods and properties as needed
}