package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;

public class MotorUtil {

    public static SparkMax createSparkMAX(int elevatorMotorId, ElevatorConstants motorType, int currentLimit, boolean inverted,
            boolean brakeMode, double rampRate) {
        SparkMax sparkMax = new SparkMax(elevatorMotorId, motorType.kBrushless);
        return sparkMax;
    }
}