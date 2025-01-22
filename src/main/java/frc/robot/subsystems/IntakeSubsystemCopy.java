// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemCopy extends SubsystemBase {

    
        @SuppressWarnings("unused")
        final private PWMSparkMax m_rightIntakeMotor = new PWMSparkMax(0); 
        @SuppressWarnings("unused")
        final private PWMSparkMax m_leftIntakeMotor = new PWMSparkMax(0);
        @SuppressWarnings("unused")
        final private DoubleSolenoid m_leftDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        @SuppressWarnings("unused")
        final private DoubleSolenoid m_rightDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);

}