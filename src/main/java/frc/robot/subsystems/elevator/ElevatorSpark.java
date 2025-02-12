
package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;


public class ElevatorSpark {


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


    public class MotorType {
        public enum Type {
            kBrushless,
            kBrushed
        }
        public static com.revrobotics.spark.SparkLowLevel.MotorType kBrushless;
    }
    
    public class MotorUtil {

    public static ElevatorSpark createSparkMAX(int elevatorMotorId, Constants motorType, int currentLimit, boolean inverted,
            boolean brakeMode, double rampRate) {
        ElevatorSpark sparkMax = new ElevatorSpark();
        return sparkMax;
    }
}


}


