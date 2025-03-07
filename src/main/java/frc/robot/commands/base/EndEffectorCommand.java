// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Sensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  private final EndEffector endEffector;
  private final double voltage;
  private final Elevator elevator;
  private final Sensors sensors;
  private final Dashboard dashboard;
  private final ActionController controller;
  /** Creates a new DefaultEndEffector. */
  public EndEffectorCommand(EndEffector endEffector, double voltage, Elevator elevator, Sensors sensors, Dashboard dashboard, ActionController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
    this.voltage = voltage;
    this.elevator = elevator;
    this.sensors = sensors;
    this.dashboard = dashboard;
    this.controller = controller;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.atTargetPosition() && elevator.getCurrentHeight() > 10) {
      endEffector.setVoltage(voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);
    dashboard.clearSelections();
    controller.toggleAutoMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!sensors.getOuttakeLaserBroken()) {
      return true;
    }
    return false;
  }
}
