// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

    
    
        final private SparkMax m_rightIntakeMotor = new SparkMax(20, SparkMax.MotorType.kBrushless); 
        final private SparkMax m_leftIntakeMotor = new SparkMax(21, SparkMax.MotorType.kBrushless);
    
      
        private  SparkMaxConfig   m_leftCfg   = new SparkMaxConfig();
        private  SparkMaxConfig   m_rightCfg   = new SparkMaxConfig();
    
        public void startIntake()  {
            m_leftIntakeMotor.setVoltage(6);
            m_rightIntakeMotor.setVoltage(6);
            

        }
        public void stopIntake(){
            m_leftIntakeMotor.setVoltage(0);
            m_rightIntakeMotor.setVoltage(0);
        }
        
        public GripperSubsystem(){
            m_leftCfg.voltageCompensation(12);
            m_leftCfg.follow(m_rightIntakeMotor);
            m_leftIntakeMotor.configure(m_leftCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            m_rightCfg.voltageCompensation(12);
            m_rightIntakeMotor.configure(m_rightCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            





        }
}
