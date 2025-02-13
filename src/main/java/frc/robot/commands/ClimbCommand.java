package frc.robot.commands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ClimbCommand extends WaitCommand {
private final ClimbSubsystem m_climbSubsystem;
WaitCommand waitCommand = new WaitCommand(1);
public  ClimbCommand(ClimbSubsystem ClimbSubsystem) {
    super(1); // Call the superclass constructor with a duration
   this.m_climbSubsystem = ClimbSubsystem;
}

public void initialize() {
    m_climbSubsystem.Climb();
  }
  public void end(boolean interrupted) {
   
  }
  public boolean isFinished() {
    m_climbSubsystem.StopMotor();
        return false;
  }

public void execute() {
  }
public void end() {
  
}
}

