package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
=======
import frc.robot.Constants;
>>>>>>> 070219f9177d1e24969b5243ff8ee51c979be545
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ElevatorConstants;




public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotorController;
   DutyCycleEncoder elevatorEncodor = new DutyCycleEncoder(30);
<<<<<<< HEAD
    public static frc.robot.subsystems.elevator.ElevatorSpark elevatorMotorController;
    private RelativeEncoder elevatorEncoder;
    public ElevatorSubsystem() {
    
          m_elevatorMotorController = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless); 
=======
    public static frc.robot.subsystems.elevator.SparkMax elevatorMotorController;
    public static RelativeEncoder elevatorEncoder;
    public ElevatorSubsystem() {
    
          m_elevatorMotorController = new SparkMax(22, MotorType.kBrushless); 
>>>>>>> 070219f9177d1e24969b5243ff8ee51c979be545
        elevatorEncoder = m_elevatorMotorController.getEncoder();
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
        m_elevatorMotorController.set(speed);
    }

    public void setEncoderPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    public double getElevatorCurrent() {
        return m_elevatorMotorController.getOutputCurrent();
    }

    public void periodicUpdate() {
        // Only code in here that relates to a physical subsystem
        SmartDashboard.putNumber("elevator/Real motor temp (C)", m_elevatorMotorController.getMotorTemperature());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        periodicUpdate();
    }
    public void raiseElevator(){
    m_elevatorMotorController.set(1.0);
    if (elevatorEncoder.getPosition() >= 0.75) {
        m_elevatorMotorController.set(0.0);
    }
}
        public void lowerElevator(){
        m_elevatorMotorController.set(-1.0);
        if (elevatorEncoder.getPosition()<=0.0){
            m_elevatorMotorController.set(0.0);
        }
    }
<<<<<<< HEAD
} 
=======
}
>>>>>>> 070219f9177d1e24969b5243ff8ee51c979be545
