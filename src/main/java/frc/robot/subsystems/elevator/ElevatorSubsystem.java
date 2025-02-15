package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.PWM1Configs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.math.ElevatorMath;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSpark;
import frc.robot.utils.Cache;
import frc.robot.utils.PIDFConfig;
import frc.robot.sensors.CANdiEncoder;




public class ElevatorSubsystem extends SubsystemBase {
    public final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(3, 3, new Color8Bit(Color.kBlack));
    private final LoggedMechanismRoot2d mechRoot2d = mechanism2d.getRoot("Elevator Root", 1.5, 0);
    private final LoggedMechanismLigament2d elevatorMech2d = mechRoot2d.append(new LoggedMechanismLigament2d("Elevator", ElevatorConstants.ELEVATOR_INITIAL_HEIGHT.in(Meters), 90.0, 50, new Color8Bit(Color.kBlue)));
    private final ElevatorSpark m_elevatorMotor;
    private SparkBaseConfig elevatorcfg = new SparkMaxConfig();
    private final CANdiEncoder m_elevatorEncoder;
    private final PWM1Configs m_elevatorPWMConfig = new PWM1Configs();
    public final PIDFConfig elevatorPIDFConfig;
    private ElevatorFeedforward elevatorFeedforward;
    public final Cache<Distance> elevatorPositionCache;
    public final Cache<LinearVelocity> elevatorVelocityCache;


    public ElevatorSubsystem(CANdi elevatorCANdi) {
    
        m_elevatorMotor = new ElevatorSpark(new SparkFlex(22, MotorType.kBrushless), elevatorcfg, DCMotor.getNeoVortex(1)); 
        m_elevatorPWMConfig.withAbsoluteSensorDiscontinuityPoint(ElevatorConstants.ELEVATOR_ABSOLUTE_SENSOR_DISCONTINUITY_POINT);
        m_elevatorPWMConfig.withAbsoluteSensorOffset(ElevatorConstants.ELEVATOR_ABSOLUTE_SENSOR_OFFSET);
        m_elevatorPWMConfig.withSensorDirection(ElevatorConstants.ELEVATOR_ENCODER_IS_INVERTED);
        elevatorCANdi.getConfigurator().apply(m_elevatorPWMConfig);
        m_elevatorEncoder = new CANdiEncoder(elevatorCANdi, 1);
        elevatorFeedforward = ElevatorMath.createElevatorFeedforward();

        elevatorPIDFConfig = new PIDFConfig(ElevatorConstants.ELEVATOR_P,
                                            ElevatorConstants.ELEVATOR_I,
                                            ElevatorConstants.ELEVATOR_D,
                                            ElevatorConstants.ELEVATOR_FF,
                                            ElevatorConstants.ELEVATOR_IZ
                                            );

        m_elevatorMotor.setVoltageCompensation(Constants.NOMINAL_VOLTAGE);
        m_elevatorMotor.setCurrentLimit(ArmConstants.SHOULDER_MOTOR_CURRENT_LIMIT);
        m_elevatorMotor.setLoopRampRate(ArmConstants.SHOULDER_MOTOR_RAMP_RATE);
        m_elevatorMotor.setInverted(ArmConstants.SHOULDER_MOTOR_IS_INVERTED);
        m_elevatorMotor.setMotorBrake(true);
        m_elevatorMotor.burnFlash();

        elevatorPositionCache = new Cache<>(m_elevatorEncoder::getPosition, 20);
        elevatorVelocityCache = new Cache<>(m_elevatorEncoder::getVelocity, 20);
        elevatorPositionCache.update();
        elevatorVelocityCache.update();

        // elevatorEncoder.setPositionConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
        // dividing by 60 to convert meters per minute to meters per second
        // elevatorEncoder.setVelocityConversionFactor(Double.parseDouble(ElevatorConstants.METERS_PER_REVOLUTION));
    }

    public Distance getElevatorPosition() {
        return Meters.of(m_elevatorEncoder.getPosition().in(Rotations) * ElevatorConstants.ELEVATOR_CONVERSION_FACTOR.in(Meters));
    }
    public LinearVelocity getElevatorVelocity() {
        return MetersPerSecond.of(m_elevatorEncoder.getVelocity().in(RotationsPerSecond) * ElevatorConstants.ELEVATOR_CONVERSION_FACTOR.in(Meters));
    }

    public void setElevatorPosition(Distance setpoint) {
        m_elevatorMotor.setReference(setpoint.in(Meters), elevatorFeedforward.calculate(0.0));
    }

    public double getElevatorCurrent() {
        return m_elevatorMotor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        periodicUpdate();
    }
    
    
}
