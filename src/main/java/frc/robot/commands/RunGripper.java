package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.gripper.GripperSubsystem;



public class RunGripper extends Command {
    private final GripperSubsystem m_GripperSubsystem;
    private CommandXboxController m_controller;
    boolean finished;


    public void setIntakeFromTrigger(CommandXboxController controller)
        {
            
        }

    public RunGripper(GripperSubsystem gripperSubsystem, CommandXboxController controller) {
        m_GripperSubsystem = gripperSubsystem;
        m_controller = controller;
        addRequirements(m_GripperSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //m_GripperSubsystem.setIntakeSpeed(setpoint);

        if (m_controller.getLeftTriggerAxis() >= 0.05 && m_controller.getRightTriggerAxis() < 0.05)
        {
            m_GripperSubsystem.setIntakeSpeed(Math.pow(m_controller.getLeftTriggerAxis(),10.0));
        }
        else if (m_controller.getLeftTriggerAxis() < 0.05 && m_controller.getRightTriggerAxis() >= 0.05)
        {
            m_GripperSubsystem.setIntakeSpeed(Math.pow(m_controller.getRightTriggerAxis(), 10.0) * -1);
        }
        else
        {
            m_GripperSubsystem.setIntakeSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
