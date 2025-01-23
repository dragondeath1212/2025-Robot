// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemCopy extends SubsystemBase {

    
    
        final private SparkMax m_rightIntakeMotor = new SparkMax(0, SparkMax.MotorType.kBrushless); 
        final private SparkMax m_leftIntakeMotor = new SparkMax(0, SparkMax.MotorType.kBrushless);
    
        final private DoubleSolenoid m_leftDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        final private DoubleSolenoid m_rightDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        final private DoubleSolenoid m_extenderDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,0);
      boolean IsIntakeClosed = false;

      public void extendIntake(){
            m_extenderDoubleSolenoid.set(Value.kForward);
      }
      public void returnIntake(){
        m_extenderDoubleSolenoid.set(Value.kReverse);
  }


       public void closeIntake(){
            if (IsIntakeClosed == false){
            m_leftDoubleSolenoid.set(Value.kForward);
            m_rightDoubleSolenoid.set(Value.kForward);
             IsIntakeClosed = true;
            }
        }

        public void openIntake(){
            if (IsIntakeClosed == true){
                
                
            m_leftDoubleSolenoid.set(Value.kReverse);
            m_rightDoubleSolenoid.set(Value.kReverse);
            IsIntakeClosed = false;
            }
            }
        public void startIntake()  {
            m_leftIntakeMotor.set(1);
            m_rightIntakeMotor.set(1);
            

        }
        public void stopIntake(){
            m_leftIntakeMotor.set(0);
            m_rightIntakeMotor.set(0);
        }
        @SuppressWarnings("deprecation")
        public IntakeSubsystemCopy(){
           
            m_rightIntakeMotor.setInverted(false);
            m_leftIntakeMotor.setInverted(false);


            m_leftDoubleSolenoid.set(Value.kOff);
            m_rightDoubleSolenoid.set(Value.kOff);





        }
}
