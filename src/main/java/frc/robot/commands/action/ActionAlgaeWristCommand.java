// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.AlgaeWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionAlgaeWristCommand extends Command {
  /** Creates a new DefaultAlgaeWrist. */
  private final AlgaeWrist algaeWrist;
  private final ActionController controller;
  public ActionAlgaeWristCommand(AlgaeWrist algaeWrist, ActionController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeWrist = algaeWrist;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Object angle = controller.getCommandField(Constants.MapConstants.WRIST_POSITION);
    if (angle != null) {
      algaeWrist.setPosition((double) angle);
    }
    else {
      algaeWrist.setPosition(Constants.AlgaeWristConstants.MIN_POSITION);
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
