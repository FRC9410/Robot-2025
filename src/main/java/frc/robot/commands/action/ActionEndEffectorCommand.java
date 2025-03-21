// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionEndEffectorCommand extends Command {
  private final EndEffector endEffector;
  private final ActionController controller;
  /** Creates a new DefaultEndEffector. */
  public ActionEndEffectorCommand(EndEffector endEffector, ActionController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.endEffector = endEffector;
    this.controller = controller;

    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object voltage = controller.getCommandField(Constants.MapConstants.END_EFFECTOR_VOLTAGE);
    if (voltage != null) {
      // Use the speed variable as needed
      endEffector.setVoltage((double) voltage);
    } else {
      endEffector.setVoltage(Constants.EndEffectorConstants.STOP_VOLTAGE);
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
