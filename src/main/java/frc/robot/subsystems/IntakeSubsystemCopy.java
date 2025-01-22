// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemCopy extends SubsystemBase {

    
        @SuppressWarnings("unused")
        final private SparkMax m_rightIntakeMotor = new SparkMax(0, SparkMax.MotorType.kBrushless); 
        @SuppressWarnings("unused")
        final private SparkMax m_leftIntakeMotor = new SparkMax(0, SparkMax.MotorType.kBrushless);
        @SuppressWarnings("unused")
        final private DoubleSolenoid m_leftDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        @SuppressWarnings("unused")
        final private DoubleSolenoid m_rightDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        public IntakeSubsystemCopy(){
            m_rightIntakeMotor.setInverted(false);
            m_leftIntakeMotor.setInverted(false);


            m_leftDoubleSolenoid.set(Value.kOff);
            m_rightDoubleSolenoid.set(Value.kOff);




        }
}
