// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ActionController.Action;
import frc.robot.subsystems.Subsystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionRequestCommand extends Command {
  private final Action action;
  private final Subsystems subsystems;
  /** Creates a new ActionRequestCommand. */
  public ActionRequestCommand(Subsystems subsystems, Action action) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystems = subsystems;
    this.action = action;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystems.getActionController().doRequest(subsystems.getSubsystemData(), action);
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
