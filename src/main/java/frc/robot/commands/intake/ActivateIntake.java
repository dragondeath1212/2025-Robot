package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystemCopy; 

public class ActivateIntake extends Command {
private final IntakeSubsystemCopy intakeSubsystem;





public ActivateIntake(IntakeSubsystemCopy intakeSubsystem) {

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