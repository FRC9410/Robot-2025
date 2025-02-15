// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultAlgaeWristCommand extends Command {
  /** Creates a new DefaultAlgaeWrist. */
  private AlgaeWrist algaeWrist;
  private Function<String, Object> commandData;
  public DefaultAlgaeWristCommand(AlgaeWrist algaeWrist, Function<String, Object> commandData) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeWrist = algaeWrist;
    this.commandData = commandData;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Object angle = commandData.apply(Constants.MapConstants.WRIST_ANGLE);
    if (angle != null) {
      algaeWrist.setAngle((double) angle);
    }
    else {
      algaeWrist.setAngle(Constants.AlgaeWristConstants.MIN_ANGLE_UNITS);
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
