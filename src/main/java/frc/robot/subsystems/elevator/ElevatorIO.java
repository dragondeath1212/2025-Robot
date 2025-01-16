package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    void setMotorSpeed(double speed);
    double getEncoderPosition();
    double getEncoderSpeed();
    void setEncoderPosition(double position);
    double getElevatorCurrent();
    void periodicUpdate();
}