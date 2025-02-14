package frc.robot.commands.ClimbComands;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;



public class ClimbCommand extends Command {
private final ClimbSubsystem m_climbSubsystem;
public  ClimbCommand(ClimbSubsystem ClimbSubsystem) {
   this.m_climbSubsystem = ClimbSubsystem;
}

public void initialize() {
    m_climbSubsystem.Climb();
  }
  public void end(boolean interrupted) {
   
  }
  public boolean isFinished() {
    
        return false;
  }

public void execute() {
  }
public void end() {
  m_climbSubsystem.StopMotor();
}
}

