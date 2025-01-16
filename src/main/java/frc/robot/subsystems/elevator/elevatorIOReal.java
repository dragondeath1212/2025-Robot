package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 

public class elevatorIOReal implements ElevatorIO {

    public static CANSparkMax elevatorMotorController;
    public static RelativeEncoder elevatorEncoder;

    public elevatorIOReal()
    {
        elevatorMotorController = MotorUtil.createSparkMAX(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless, 
            Constants.NEO_CURRENT_LIMIT, false, true, 0.1);
        
        elevatorEncoder = elevatorMotorController.getEncoder();
        // elevatorEncoder.setPositionConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
        // dividng by 60 to convert meters per miniute to meters per seconds
        // elevatorEncoder.setVelocityConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
    }

    @Override
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }
    public double getEncoderSpeed() {
        return elevatorEncoder.getVelocity();
    }
    
    @Override
    public void setMotorSpeed(double speed) {
        frc.robot.subsystems.elevator.elevatorMotorController.set(speed);
    }

    @Override
    public void setEncoderPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    @Override
    public double getElevatorCurrent() {
        return frc.robot.subsystems.elevator.elevatorMotorController.getOutputCurrent();
    }

    @Override
    public void periodicUpdate() {
        // Only code in here that relates a physical subsystem
        SmartDashboard.putNumber("elevator/Real motor temp (C)", frc.robot.subsystems.elevator.elevatorMotorController.getMotorTemperature());
    }

}