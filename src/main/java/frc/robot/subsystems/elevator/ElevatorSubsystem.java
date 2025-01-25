package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkMax;
import com.revrobotics.RelativeEncoder;

@SuppressWarnings("unused")
public class ElevatorSubsystem extends SubsystemBase {
    public static frc.robot.subsystems.elevator.SparkMax elevatorMotorController;
    public static RelativeEncoder elevatorEncoder;
    public ElevatorSubsystem() {
    
         elevatorMotorController = MotorUtil.createSparkMAX(ElevatorConstants.ELEVATOR_MOTOR_ID, ElevatorConstants.kBrushless, 
            1, false, true, 0.1); 
        elevatorEncoder = elevatorMotorController.getEncoder();
        // elevatorEncoder.setPositionConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
        // dividing by 60 to convert meters per minute to meters per second
        // elevatorEncoder.setVelocityConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
    }
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    public double getEncoderSpeed() {
        return elevatorEncoder.getVelocity();
    }

    public void setMotorSpeed(double speed) {
        elevatorMotorController.set(speed);
    }

    public void setEncoderPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    public double getElevatorCurrent() {
        return elevatorMotorController.getOutputCurrent();
    }

    public void periodicUpdate() {
        // Only code in here that relates to a physical subsystem
        SmartDashboard.putNumber("elevator/Real motor temp (C)", elevatorMotorController.getMotorTemperature());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        periodicUpdate();
    }
    public void raiseElevator(){
    elevatorMotorController.set(0.5);
    if( elevatorEncoder.getPosition()>=0.75){
        elevatorMotorController.set(0.0);
    }
}
        public void lowerElevator(){
        elevatorMotorController.set(-0.5);
        if (elevatorEncoder.getPosition()<=0.0){
            elevatorMotorController.set(0.0);
        }
    }
}