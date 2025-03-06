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
public class ElevatorAlgaeCommand extends Command {
  private final Elevator elevator;
  private final Dashboard dashboard;
  private final Sensors sensors;

  /** Creates a new DefaultElevator. */
  public ElevatorAlgaeCommand(Elevator elevator, Sensors sensors, Dashboard dashboard) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.dashboard = dashboard;
    this.sensors = sensors;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!sensors.getIntakeLaserBroken()) {
      elevator.setPosition(dashboard.getSelectedCoralLevel());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPosition(Constants.ElevatorConstants.HOME_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
