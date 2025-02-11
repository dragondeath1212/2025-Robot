// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    
    
        final private SparkMax m_rightIntakeMotor = new SparkMax(20, SparkMax.MotorType.kBrushless); 
        final private SparkMax m_leftIntakeMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    
        final private DoubleSolenoid m_leftDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        final private DoubleSolenoid m_rightDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
        final private DoubleSolenoid m_extenderDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4,5);
        private  SparkMaxConfig   m_leftCfg   = new SparkMaxConfig();
        private  SparkMaxConfig   m_rightCfg   = new SparkMaxConfig();
    

      public void extendIntake(){
            m_extenderDoubleSolenoid.set(Value.kForward);
      }
      public void returnIntake(){
        m_extenderDoubleSolenoid.set(Value.kReverse);
  }


       public void closeIntake(){
            
            m_leftDoubleSolenoid.set(Value.kForward);
            m_rightDoubleSolenoid.set(Value.kForward);
             
            
        }

        public void openIntake(){
            
                
                
            m_leftDoubleSolenoid.set(Value.kReverse);
            m_rightDoubleSolenoid.set(Value.kReverse);
            
            }
        public void startIntake()  {
            m_leftIntakeMotor.setVoltage(6);
            m_rightIntakeMotor.setVoltage(6);
            

        }
        public void stopIntake(){
            m_leftIntakeMotor.setVoltage(0);
            m_rightIntakeMotor.setVoltage(0);
        }
        @SuppressWarnings("deprecation")
        public IntakeSubsystem(){
            m_leftCfg.voltageCompensation(12);
            m_leftCfg.follow(m_rightIntakeMotor);
            m_leftIntakeMotor.configure(m_leftCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            m_rightCfg.voltageCompensation(12);
            m_rightIntakeMotor.configure(m_rightCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            m_rightIntakeMotor.setInverted(true);
            m_leftIntakeMotor.setInverted(false);






        }
}
