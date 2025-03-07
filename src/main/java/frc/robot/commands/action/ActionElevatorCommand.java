// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionElevatorCommand extends Command {
  private final Elevator elevator;
  private final ActionController controller;
  /** Creates a new DefaultElevator. */
  public ActionElevatorCommand(Elevator elevator, ActionController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.controller = controller;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object position = controller.getCommandField(Constants.MapConstants.ELEVATOR_POSITION);
    if (position != null) {
      // Use the speed variable as needed
      elevator.setPosition((double) position);
    } else {
      elevator.setPosition(Constants.ElevatorConstants.HOME_POSITION);
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
