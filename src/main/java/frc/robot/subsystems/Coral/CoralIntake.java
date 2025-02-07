// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.SimulatableSparkMax;

public class CoralIntake extends SubsystemBase {

  SimulatableSparkMax intakeMotor;

  public static CoralIntake instance;

  public static CoralIntake getInstance() {
    if(instance == null) {
      instance = new CoralIntake();
    }
    return instance;
  }

  /** Creates a new Coral. */
  public CoralIntake() {
    intakeMotor = new SimulatableSparkMax(CoralConstants.kCoralPort, MotorType.kBrushed);

    intakeMotor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(CoralConstants.smartCurrentLimit).voltageCompensation(CoralConstants.voltageCompensation);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Logger.recordOutput("Coral Intake", "Coral Intake olusturuldu.");
  }

  public void score() {
    intakeMotor.set(CoralConstants.scoreSpeed);
    Logger.recordOutput("Coral Intake", "Coral Intake su hizda calisiyor: " + CoralConstants.scoreSpeed);
  }

  public void stop() {
    intakeMotor.stopMotor();
    Logger.recordOutput("Coral Intake", "Coral Intake durduruldu.");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
