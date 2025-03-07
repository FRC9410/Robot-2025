// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Sensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultEndEffectorCommand extends Command {
  private final EndEffector endEffector;
  private final Sensors sensors;
  private final Elevator elevator;
  /** Creates a new DefaultEndEffector. */
  public DefaultEndEffectorCommand(EndEffector endEffector, Sensors sensors, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
    this.sensors = sensors;
    this.elevator = elevator;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sensors.getIntakeLaserBroken() && sensors.getOuttakeLaserBroken()) {
      endEffector.setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE/2);
    }
    else if (sensors.getIntakeLaserBroken() && !sensors.getOuttakeLaserBroken()) {
      endEffector.setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE);
    } else {
      endEffector.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);
    }

    if (elevator.atTargetPosition() && elevator.getCurrentHeight() > 10) {
      endEffector.setVoltage(Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
