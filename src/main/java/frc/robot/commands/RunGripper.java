package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.gripper.GripperSubsystem;



public class RunGripper extends Command {
    private final GripperSubsystem m_GripperSubsystem;
    private double setpoint;
    boolean finished;


    public RunGripper(GripperSubsystem gripperSubsystem, double setpoint) {
        System.out.println("In rungripper ctor");
        m_GripperSubsystem = gripperSubsystem;
        this.setpoint = setpoint;
        addRequirements(m_GripperSubsystem);
    }
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_GripperSubsystem.setIntakeSpeed(setpoint);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
