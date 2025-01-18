package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

/*Begins the class declaration. */

public class ArmSubsystem {
   /*Creates a position, current cycle position, pre-incline up, and power reading. */ 
    @SuppressWarnings("unused")
    private double position;
    @SuppressWarnings("unused")
    private int currCyclePos;
    @SuppressWarnings("unused")
    private boolean preIncUp;
    @SuppressWarnings("unused")
    private double power;
/*States position */
    @SuppressWarnings("unused")
    private double[] positions = { 0, 1441 }; // in degree// First value is reset Pos
    /*Moves the arm up and down. */
    public static final double POWER_DOWN = 0.3;
    public static final double POWER_UP = -0.4;
    @SuppressWarnings("unused")
    private final double CONVERSION_FACTOR = 1024.0 / 360.0;
    @SuppressWarnings("unused")
    private static final double SHOOTER_COLLISION_ANGLE = 100;
/*Declares P, I, and, D. */
    private final double ARM_P = 1.0;
    private final double ARM_I = 0;
    private final double ARM_D = 0;

    private TalonFX armMotor;
    private TalonFXConfiguration armConfig;

    @SuppressWarnings("removal")
    public ArmSubsystem() {
        armMotor = new TalonFX(1); // Update with the correct CAN ID
        armConfig = new TalonFXConfiguration();

        armConfig.Slot0.kP = ARM_P;
        armConfig.Slot1.kI = ARM_I;
        armConfig.Slot2.kD = ARM_D;

        armMotor.getConfigurator().apply(armConfig);
        armMotor.setInverted(true); // Assuming CounterClockwise_Positive means true
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.set(0); // Replace 0 with the desired position setpoint
    }
}