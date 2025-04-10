// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Sensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPositionCommand extends Command {
  private final Elevator elevator;
  private final double position;
  private final Sensors sensors;
  private boolean override;

  /** Creates a new DefaultElevator. */
  public ElevatorPositionCommand(Elevator elevator, Sensors sensors, double position, boolean override) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.position = position;
    this.sensors = sensors;
    this.override = override;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((sensors.getOuttakeLaserBroken() && !sensors.getIntakeLaserBroken()) || position == Constants.ElevatorConstants.HOME_POSITION || override) {
      elevator.setPosition(position);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!sensors.getOuttakeLaserBroken()) {
      elevator.setPosition(Constants.ElevatorConstants.HOME_POSITION);
    } else if (!elevator.atTargetPosition()) {
      elevator.setPosition(Constants.ElevatorConstants.HOME_POSITION);
    } else if (override) {
      elevator.setPosition(Constants.ElevatorConstants.HOME_POSITION);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!sensors.getOuttakeLaserBroken() && !override) {
      return true;
    }
    return false;
  }
}
