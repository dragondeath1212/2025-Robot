// Author: Innovators robotics
// Last Updated : today

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

    
    
        final private SparkMax m_feederMotor = new SparkMax(20, SparkMax.MotorType.kBrushless); 
        final private AnalogInput m_lightsensor = new AnalogInput(0);
      
        private  SparkMaxConfig   m_feederCfg   = new SparkMaxConfig();
    
        public void startIntake()  {
            m_feederMotor.set(6);
            

        }
        public void stopIntake(){
            m_feederMotor.set(0);
        }
        public int getLightSensorValue() {

            return m_lightsensor.getAverageValue();
        }
        
        public GripperSubsystem(){
            m_feederCfg.voltageCompensation(12);
            m_feederCfg.follow(m_feederMotor);
            m_feederMotor.configure(m_feederCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            m_feederCfg.voltageCompensation(12);
            m_feederMotor.configure(m_feederCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


            





        }
}
