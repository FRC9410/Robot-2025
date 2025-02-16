// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionClimberCommand extends Command {
  private final Climber climber;
  private final ActionController controller;
  /** Creates a new DefaultClimber. */
  public ActionClimberCommand(Climber climber, ActionController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.controller = controller;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object climberPosition = controller.getCommandField(Constants.MapConstants.CLIMBER_POSITION);
    if(climberPosition != null) {
      climber.setPosition((Double) climberPosition);
    }
    else {
      climber.setPosition(Constants.ClimberConstants.CLIMBER_DEFAULT_POSITION);
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
