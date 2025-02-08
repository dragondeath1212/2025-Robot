package frc.robot.subsystems.arm;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ArmSpark {

    private static final int maximumRetries = 5;
    private SparkBaseConfig cfg;
    private final SparkBase motor;
    private final DCMotor motorType;

    public ArmSpark(SparkBase motor, SparkBaseConfig cfg, DCMotor motorType)
    {
        this.motor = motor;
        this.cfg = cfg;
        this.motorType = motorType;
        configureSpark(motor::clearFaults);
    }

    public void configureSpark(Supplier<REVLibError> config)
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

    public void updateConfig(SparkBaseConfig motorCfg, SparkBaseConfig cfgGiven)
    {
        if (!DriverStation.isDisabled())
        {
            throw new RuntimeException("Configuration changes cannot be applied while the robot is enabled.");
        }
        motorCfg.apply(cfgGiven);
        configureSpark(() -> motor.configure(motorCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    }

    public void setVoltageCompensation(double nominalVoltage)
    {
        cfg.voltageCompensation(nominalVoltage);
    }

    public void setCurrentLimit(int currentLimit)
    {
        cfg.smartCurrentLimit(currentLimit);
    }

    public void setLoopRampRate(double rampRate)
    {
        cfg.closedLoopRampRate(rampRate)
           .openLoopRampRate(rampRate);
    }

    public Object getMotor()
    {
        return motor;
    }



}

