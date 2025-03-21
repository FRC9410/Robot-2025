// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.Utils;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MapConstants;

public class Sensors extends SubsystemBase {
private final BiConsumer<String, Object> updateData;
  private final LaserCan intakeLaser;
  private final LaserCan outtakeLaser;
  /** Creates a new Sensors. */
  public Sensors(BiConsumer<String, Object> updateData) {
    intakeLaser = new LaserCan(Constants.SensorConstants.INTAKE_LASER_CAN_ID);
    outtakeLaser = new LaserCan(Constants.SensorConstants.OUTTAKE_LASER_CAN_ID);
    
    this.updateData = updateData;

    
    if (!Utils.isSimulation()) {
      try {
        intakeLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        intakeLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 16, 2, 2));
        intakeLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }

  
      try {
        outtakeLaser.setRangingMode(LaserCan.RangingMode.SHORT);
        outtakeLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
        outtakeLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
      }
    }
}
  
  @Override
  public void periodic() {
    updateData.accept(MapConstants.HAS_PIECE, getOuttakeLaserBroken() && !getIntakeLaserBroken());
  }

  
  private double getIntakeLaserMeasurement() {
    return intakeLaser.getMeasurement().distance_mm;
  }

  
  private double getOuttakeLaserMeasurement() {
    return intakeLaser.getMeasurement().distance_mm;
  }
  
  public boolean getIntakeLaserBroken() {
    if (Utils.isSimulation()) {
      return false;
    }

    double measurement = intakeLaser.getMeasurement().distance_mm;
    return measurement <= Constants.SensorConstants.INTAKE_BREAKBEAM;
  }
  
  public boolean getOuttakeLaserBroken() {
    if (Utils.isSimulation()) {
      return false;
    }
    
    double measurement = outtakeLaser.getMeasurement().distance_mm;
    return measurement <= Constants.SensorConstants.OUTTAKE_BREAKBEAM;
  }
}
