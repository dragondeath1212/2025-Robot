package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.ControlType;

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

    private CANSparkMax armMotor;
    private SparkMaxPIDController pidController;
    private CANEncoder encoder;

    public ArmSubsystem() {
        armMotor = new CANSparkMax(); // Update with the correct CAN ID
        pidController = armMotor.getPIDController();
        pidController.setReference(0, ControlType.kPosition);
        encoder = armMotor.getEncoder();

        armMotor.getPIDController().setP(ARM_P);
        armMotor.getPIDController().setI(ARM_I);
        armMotor.getPIDController().setD(ARM_D);
        encoder.setPosition(0);
    }
}