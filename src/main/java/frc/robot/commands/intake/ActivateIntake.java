//Activates Intake
package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem; 

public class ActivateIntake extends Command {
private final IntakeSubsystem intakeSubsystem;

private boolean m_finished =  false;

    public ActivateIntake(IntakeSubsystem intakeSubsystem) {


this.intakeSubsystem = intakeSubsystem;


}

    public void initialize() {
       
    }

    public void execute() {
        if (!m_finished) {
            intakeSubsystem.extendIntake();
            intakeSubsystem.openIntake();
            intakeSubsystem.startIntake();
            m_finished = true;
        }
    }

    public void end() {

    }

    public boolean isfinished() {
        return m_finished;
    }
}