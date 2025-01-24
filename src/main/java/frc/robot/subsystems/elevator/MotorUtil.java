package frc.robot.subsystems.elevator;

public class MotorUtil {

    public static SparkMax createSparkMAX(int elevatorMotorId, MotorType motorType, int currentLimit, boolean inverted,
            boolean brakeMode, double rampRate) {
        SparkMax sparkMax = new SparkMax(elevatorMotorId);
        sparkMax.setMotorType(motorType);
        sparkMax.setCurrentLimit(currentLimit);
        sparkMax.setInverted(inverted);
        sparkMax.setBrakeMode(brakeMode);
        sparkMax.setRampRate(rampRate);
        return sparkMax;
    }

    public static SparkMax createSparkMAX(int elevatorMotorId, MotorType kbrushless, String neoCurrentLimit,
            boolean inverted, boolean brakeMode, double rampRate) {
        
        throw new UnsupportedOperationException("Unimplemented method 'createSparkMAX'");
    }
}