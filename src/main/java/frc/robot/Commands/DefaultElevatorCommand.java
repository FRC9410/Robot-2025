// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultElevatorCommand extends Command {
  private final Elevator elevator;
  private final Function<String, Object> commandData;
  /** Creates a new DefaultElevator. */
  public DefaultElevatorCommand(Elevator elevator, Function<String, Object> commandData) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.commandData = commandData;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object height = commandData.apply(Constants.MapConstants.ELEVATOR_HEIGHT);
    if(height != null) {
      elevator.setHeight((double) height);
    } else {
      elevator.setHeight(Constants.ElevatorConstants.ELEVATOR_DEFAULT_HEIGHT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
