package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class DeactivateIntake extends Command {
private final IntakeSubsystem intakeSubsystem;





public DeactivateIntake(IntakeSubsystem intakeSubsystem) {

this.intakeSubsystem = intakeSubsystem;


}

public void initialize() {
    intakeSubsystem.stopIntake();
    intakeSubsystem.closeIntake();
    intakeSubsystem.returnIntake();
}
public void execute() {
    
}
public void end() {

}
}