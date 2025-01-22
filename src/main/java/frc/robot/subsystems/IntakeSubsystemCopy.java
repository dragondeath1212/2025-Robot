// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemCopy extends SubsystemBase {
  final private PWMSparkMax m_rightIntakeMotor = new PWMSparkMax(1); 
}