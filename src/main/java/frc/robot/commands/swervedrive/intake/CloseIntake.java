

package frc.robot.commands.swervedrive.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystemCopy; 

public class CloseIntake extends Command {
private final IntakeSubsystemCopy intakeSubsystem;





public CloseIntake(IntakeSubsystemCopy intakeSubsystem) {
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
