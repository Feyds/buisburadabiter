// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Algae;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.SimulatableSparkMax;

public class Algae extends SubsystemBase {

  SimulatableSparkMax motor;
  RelativeEncoder encoder;

  public static Algae instance;

  public static Algae getInstance() {
    if(instance == null) {
      instance = new Algae();
    }
    return instance;
  }

  /** Creates a new Algae. */
  public Algae() {
    motor = new SimulatableSparkMax(AlgaeConstants.kAlgaePort, MotorType.kBrushless);
    encoder = motor.getEncoder();

    motor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(AlgaeConstants.smartCurrentLimit).voltageCompensation(AlgaeConstants.voltageCompensation);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Logger.recordOutput("Subsystem", "Algae Subsystem olusturuldu.");
  }

  public void intake() {
    motor.set(AlgaeConstants.intakeSpeed);
    Logger.recordOutput("Algae", "Algae Subsystem Intake su hizda calisiyor: " + AlgaeConstants.intakeSpeed);
  }

  public void score() {
    motor.set(AlgaeConstants.scoreSpeed);
    Logger.recordOutput("Algae", "Algae Subsystem Score su hizda calisiyor: " + AlgaeConstants.scoreSpeed);
  }

  public void stop() {
    motor.stopMotor();
    Logger.recordOutput("Algae", "Algae Subsystem durduruldu.");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Algae", "Algae Subsystem Encoder degeri: " + encoder.getPosition());
  }
}
