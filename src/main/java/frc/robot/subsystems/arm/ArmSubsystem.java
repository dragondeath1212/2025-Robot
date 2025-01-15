package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ArmSubsystem {
    
    @SuppressWarnings("unused")
    private double position;
    @SuppressWarnings("unused")
    private int currCyclePos;
    @SuppressWarnings("unused")
    private boolean preIncUp;
    @SuppressWarnings("unused")
    private double power;

    @SuppressWarnings("unused")
    private double[] positions = { 0, 1441 }; // in degree// First value is reset Pos
    
    public static final double POWER_DOWN = 0.3;
    public static final double POWER_UP = -0.4;
    @SuppressWarnings("unused")
    private final double CONVERSION_FACTOR = 1024.0 / 360.0;
    @SuppressWarnings("unused")
    private static final double SHOOTER_COLLISION_ANGLE = 100;

    private final double ARM_P = 1.0;
    private final double ARM_I = 0;
    private final double ARM_D = 0;

    private static ArmSubsystem instance;

    public static ArmSubsystem getInstance() {
        if (instance == null) {
            instance = new ArmSubsystem();
        }
        return instance;
    }
public ArmSubsystem() {
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.ARM_SUBSYSTEM).changeControlMode(TalonControlMode.PercentVbus);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).set(TalonSRXControlMode.Position, 0);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).config_kP(0, ARM_P, 10);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).config_kI(0, ARM_I, 10);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).config_kD(0, ARM_D, 10);
    ((TalonSRX) ((TalonsType) ((RobotControlWithSRX) RobotControlWithSRX.getInstance()).getTalons()).get(RobotMotorType.ARM_SUBSYSTEM)).setSensorPhase(false);
    position = 0;
    power = 0;
}

}