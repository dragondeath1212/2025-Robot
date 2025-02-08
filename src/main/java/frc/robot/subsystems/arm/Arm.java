package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

import frc.robot.Constants.ArmConstants;

import frc.robot.subsystems.arm.ArmSpark;

public class Arm extends SubsystemBase {
    private SparkBaseConfig wristcfg = new SparkFlexConfig();
    private SparkBaseConfig shouldercfg = new SparkMaxConfig();
    private final ArmSpark m_shoulderMotor;
    private final ArmSpark m_wristMotor;
    private final CANdi m_armCANdi;
    private final ArmEncoder m_shoulderEncoder;
    private final ArmEncoder m_wristEncoder;
    private static final int maximumRetries = 5;

    public Arm() {

        m_shoulderMotor = new ArmSpark(new SparkFlex(18, MotorType.kBrushless), shouldercfg, DCMotor.getNeoVortex(1));
        m_wristMotor = new ArmSpark(new SparkMax(19, MotorType.kBrushless), wristcfg, DCMotor.getNEO(1));

        m_armCANdi = new CANdi(34);

        m_shoulderEncoder = new ArmEncoder(m_armCANdi, ArmConstants.SHOULDER_ENCODER_SIGNAL); 
        m_wristEncoder = new ArmEncoder(m_armCANdi, ArmConstants.WRIST_ENCODER_SIGNAL);
    }


    public void configureSpark(Supplier<REVLibError> config, SparkBase motor)
    {
        for (int i =0; i < maximumRetries; i++)
        {
            if (config.get() == REVLibError.kOk)
            {
                return;
            }
            Timer.delay(Milliseconds.of(5).in(Seconds));
        }
        DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
    }

    public void updateConfig(SparkBase motor, SparkMaxConfig motorCfg, SparkMaxConfig cfgGiven)
    {
        if (!DriverStation.isDisabled())
        {
            throw new RuntimeException("Configuration changes cannot be applied while the robot is enabled.");
        }
        motorCfg.apply(cfgGiven);
        configureSpark(() -> motor.configure(motorCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters), motor);

    }
    

}
