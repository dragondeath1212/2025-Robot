//close Intake

package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class CloseIntake extends Command {
private final IntakeSubsystem intakeSubsystem;

private boolean m_finished = false;



public CloseIntake(IntakeSubsystem intakeSubsystem) {
	this.intakeSubsystem = intakeSubsystem;
}
public void initialize() {
	
}

public void execute() {
	if (!m_finished) {
		intakeSubsystem.closeIntake();


	}
}

public void end() {

}
public boolean isfinished() {
    return m_finished;
}
}
