//Activates Intake
package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class ActivateIntake extends Command {
private final IntakeSubsystem intakeSubsystem;





public ActivateIntake(IntakeSubsystem intakeSubsystem) {

this.intakeSubsystem = intakeSubsystem;


}

public void initialize() {
    intakeSubsystem.extendIntake();
    intakeSubsystem.openIntake();
    intakeSubsystem.startIntake();
}

public void execute() {
    
}

public void end() {

}
}