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
    private boolean m_gotPiece = false;
    private int m_intakePieceCountdown = 0;
    
    private boolean m_finished = false;;

    public IntakeGamepiece(ElevatorSubsystem elevatorSubsystem, Arm arm, GripperSubsystem gripperSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_arm = arm;
        m_gripperSubsystem = gripperSubsystem;
        addRequirements(m_elevatorSubsystem, m_arm, m_gripperSubsystem);
    }
    @Override
    public void initialize() {
        m_gotPiece = false;
        m_intakePieceCountdown = 15;
        m_finished = false;
    }

    @Override
    public void execute() {

        m_gotPiece = m_gripperSubsystem.lightSensorTripped();

        if (!m_gotPiece) //wait for piece in intake position position
        {
            m_elevatorSubsystem.setElevatorPosition(ElevatorConstants.ELEVATOR_INITIAL_HEIGHT);
            m_arm.setShoulderPosition(ArmConstants.ARM_INTAKE_ANGLES[0]);
            m_arm.setWristPosition(ArmConstants.ARM_INTAKE_ANGLES[1]);

            m_gripperSubsystem.setIntakeSpeed(GripperConstants.GRIPPER_INTAKE_SPEED);
        }
        else if (m_intakePieceCountdown > 0) //keep intaking it further into the gripper
        {
            m_intakePieceCountdown--;
            m_arm.stopAllMotionAndClearPIDInfo();
            m_elevatorSubsystem.stopAllMotionAndClearPIDInfo();
        }
        else //got piece and its far enough in
        {
            m_finished = true;
        }
        
    }

    @Override
    public void end(boolean interrupted)
    {
        m_gripperSubsystem.setIntakeSpeed(0.0);
        m_arm.stopAllMotionAndClearPIDInfo();
        m_elevatorSubsystem.stopAllMotionAndClearPIDInfo();
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
