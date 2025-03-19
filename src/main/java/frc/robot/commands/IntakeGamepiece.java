package frc.robot.commands;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.GripperSubsystem;


public class IntakeGamepiece extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Arm m_arm;
    private final GripperSubsystem m_gripperSubsystem;
    
    boolean finished;

    public IntakeGamepiece(ElevatorSubsystem elevatorSubsystem, Arm arm, GripperSubsystem gripperSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_arm = arm;
        m_gripperSubsystem = gripperSubsystem;
        //addRequirements(m_elevatorSubsystem, m_arm, m_gripperSubsystem);
        addRequirements(m_elevatorSubsystem, m_arm);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_INITIAL_HEIGHT);
        m_arm.setShoulderPosition(ArmConstants.ARM_INTAKE_ANGLES[0]);
        m_arm.setWristPosition(ArmConstants.ARM_INTAKE_ANGLES[1]);

        
        /*m_gripperSubsystem.setIntakeSpeed(GripperConstants.GRIPPER_INTAKE_SPEED);
        if (m_gripperSubsystem.lightSensorTripped()) {
            m_gripperSubsystem.setIntakeSpeed(0.0);
        }
        if (m_elevatorSubsystem.atSetpoint) {
            finished = true;
        }*/
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
