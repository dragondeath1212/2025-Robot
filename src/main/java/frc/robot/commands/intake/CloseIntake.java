

package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class CloseIntake extends Command {
private final IntakeSubsystem intakeSubsystem;





public CloseIntake(IntakeSubsystem intakeSubsystem) {
	this.intakeSubsystem = intakeSubsystem;
}
public void initialize() {
	intakeSubsystem.closeIntake();

}

public void execute() {
	
}

public void end() {

}
}
