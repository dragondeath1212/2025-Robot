//Deactivates Intake
package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class DeactivateIntake extends Command {
private final IntakeSubsystem m_intakeSubsystem;
private boolean m_finished = true;




public DeactivateIntake(IntakeSubsystem intakeSubsystem) {

this.m_intakeSubsystem = intakeSubsystem;


}

public void initialize() {

}
public void execute() {
    if (!m_finished) {
        m_intakeSubsystem.stopIntake();
   
        m_intakeSubsystem.returnIntake();
        m_finished = true;
    }



}
public void end() {


}

public boolean isfinished() {
    return m_finished;
}
}