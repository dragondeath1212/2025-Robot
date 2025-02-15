package frc.robot.math;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Constants.ElevatorConstants;


public class ElevatorMath {

    public static ElevatorFeedforward createElevatorFeedforward() {

        double ks = 0.01;
        double kg = ElevatorConstants.ELEVATOR_KG;
        double kv = ElevatorConstants.ELEVATOR_KV;
        double ka = ElevatorConstants.ELEVATOR_KA;

        return new ElevatorFeedforward(ks, kg, kv, ka);
    }
}