
package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/elevator/ElevatorSpark.java
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSpark {
========
public class SparkMax {
>>>>>>>> 070219f9177d1e24969b5243ff8ee51c979be545:src/main/java/frc/robot/subsystems/elevator/SparkMax.java

    public RelativeEncoder getEncoder() {
        throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
    }

    public void set(double speed) {
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    public double getOutputCurrent() {
        throw new UnsupportedOperationException("Unimplemented method 'getOutputCurrent'");
    }

    public double getMotorTemperature() {
        throw new UnsupportedOperationException("Unimplemented method 'getMotorTemperature'");
    }
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/elevator/ElevatorSpark.java

    public class MotorType {
        public enum Type {
            kBrushless,
            kBrushed
        }
        public static com.revrobotics.spark.SparkLowLevel.MotorType kBrushless;
    }
    
    public class MotorUtil {

    public static ElevatorSpark createSparkMAX(int elevatorMotorId, ElevatorConstants motorType, int currentLimit, boolean inverted,
            boolean brakeMode, double rampRate) {
        ElevatorSpark sparkMax = new ElevatorSpark();
        return sparkMax;
    }
}

========
>>>>>>>> 070219f9177d1e24969b5243ff8ee51c979be545:src/main/java/frc/robot/subsystems/elevator/SparkMax.java
}


