// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sensors extends SubsystemBase {
private final BiConsumer<String, Object> updateData;
  private final LaserCan intakeLaser;
  private final LaserCan outtakeLaser;
  /** Creates a new Sensors. */
  public Sensors(BiConsumer<String, Object> updateData) {
    intakeLaser = new LaserCan(Constants.SensorConstants.INTAKE_LASER_CAN_ID);
    outtakeLaser = new LaserCan(Constants.SensorConstants.OUTTAKE_LASER_CAN_ID);
    
    this.updateData = updateData;

    try {
      intakeLaser.setRangingMode(LaserCan.RangingMode.SHORT);
      intakeLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      intakeLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

 
    try {
      outtakeLaser.setRangingMode(LaserCan.RangingMode.SHORT);
      outtakeLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      outtakeLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  private double getIntakeLaserMeasurement() {
    return intakeLaser.getMeasurement().distance_mm;
  }

  
  private double getOuttakeLaserMeasurement() {
    return intakeLaser.getMeasurement().distance_mm;
  }
  
  private boolean getIntakeLaserBroken() {
    double measurement = intakeLaser.getMeasurement().distance_mm;
    return measurement >= Constants.SensorConstants.INTAKE_LASER_LOWER_BOUND
    && measurement <= Constants.SensorConstants.INTAKE_LASER_UPPER_BOUND;
  }
  
  private boolean getOuttakeLaserBroken() {
    double measurement = intakeLaser.getMeasurement().distance_mm;
    return measurement >= Constants.SensorConstants.OUTTAKE_LASER_LOWER_BOUND
    && measurement <= Constants.SensorConstants.OUTTAKE_LASER_UPPER_BOUND;
  }
}
