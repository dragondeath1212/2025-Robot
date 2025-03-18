package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.arm.Arm;


public class SetToLevelThree extends Command {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final Arm m_arm;
    
    boolean finished;

    public SetToLevelThree(ElevatorSubsystem elevatorSubsystem, Arm arm) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_arm = arm;
        addRequirements(m_elevatorSubsystem, m_arm);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_L3_HEIGHT);
        //m_arm.setShoulderPosition(ArmConstants.ARM_L3_ANGLES[0]);
        //m_arm.setWristPosition(ArmConstants.ARM_L3_ANGLES[1]);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
