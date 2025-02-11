package frc.robot.simulation;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableSparkMax extends SparkMax {
  SimDeviceSim mCANSparkMaxSim;

  SimDouble mCANSparkMaxSimAppliedOutput;

  public SimulatableSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    mCANSparkMaxSim = new SimDeviceSim("SPARK MAX ", deviceId);
    mCANSparkMaxSimAppliedOutput = mCANSparkMaxSim.getDouble("Applied Output");

    // TODO: Add other simulation fields
  }

  @Override
  public REVLibError configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
      // return super.configure(config, resetMode, persistMode);
      // Just throw away everything for now and say we're ok.
      return REVLibError.kOk;
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // TODO: Figure out why this is mad when running on a real robot
    // mCANSparkMaxSimAppliedOutput.set(speed);
  }
}



/*
package frc.robot.simulation;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimulatableSparkMax extends SparkMax {
  // Simulation device handle and simulation fields.
  private SimDeviceSim mCANSparkMaxSim;
  private SimDouble mCANSparkMaxSimAppliedOutput;
  
  // Additional simulation fields added as an example:
  private SimDouble mCANSparkMaxSimBusVoltage;
  private SimDouble mCANSparkMaxSimMotorCurrent;

  public SimulatableSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);

    // Only initialize simulation devices if we are in simulation mode.
    if (RobotBase.isSimulation()) {
      // Create a simulation device with a name that includes the device ID.
      mCANSparkMaxSim = new SimDeviceSim("SPARK MAX " + deviceId);
      
      // Existing simulated field for applied output.
      mCANSparkMaxSimAppliedOutput = mCANSparkMaxSim.getDouble("Applied Output");
      
      // TODO: Add other simulation fields.
      // For example, add a simulated bus voltage and motor current:
      mCANSparkMaxSimBusVoltage = mCANSparkMaxSim.getDouble("Bus Voltage");
      mCANSparkMaxSimMotorCurrent = mCANSparkMaxSim.getDouble("Motor Current");
    }
  }

  @Override
  public REVLibError configure(SparkBaseConfig config, ResetMode resetMode, PersistMode persistMode) {
    // For simulation purposes, we can ignore configuration changes.
    // In a more detailed simulation, you might simulate the effects of configuration.
    return REVLibError.kOk;
  }

  @Override
  public void set(double speed) {
    super.set(speed);

    // Only update simulation fields if we are in simulation.
    if (RobotBase.isSimulation() && mCANSparkMaxSimAppliedOutput != null) {
      // TODO: Figure out why this is mad when running on a real robot.
      // The simulation fields should only be updated in simulation mode.
      mCANSparkMaxSimAppliedOutput.set(speed);
      
      // Update additional simulated fields.
      // For example, assume a fixed 12V bus voltage.
      if (mCANSparkMaxSimBusVoltage != null) {
        mCANSparkMaxSimBusVoltage.set(12.0);
      }
      
      // Simulate motor current as an arbitrary function of speed (e.g., max 40A at full throttle).
      if (mCANSparkMaxSimMotorCurrent != null) {
        mCANSparkMaxSimMotorCurrent.set(Math.abs(speed) * 40.0);
      }
    }
  }
}

 */