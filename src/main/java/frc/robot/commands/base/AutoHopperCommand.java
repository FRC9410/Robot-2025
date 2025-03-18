// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoHopperCommand extends Command {
  private final Hopper hopper;
  private final Sensors sensors;
  /** Creates a new DefaultHopper. */
  public AutoHopperCommand(Hopper hopper, Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.sensors = sensors;

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sensors.getIntakeLaserBroken() || sensors.getOuttakeLaserBroken()) {
      hopper.setVoltage(HopperConstants.STOP_VOLTAGE);
    }
    else {
      hopper.setVoltage(HopperConstants.START_VOLTAGE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}