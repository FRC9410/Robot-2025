// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultAlgaeIntakeCommand extends Command {
  private AlgaeIntake algaeIntake;
  private Function<String, Object> commandData;
  /** Creates a new DefaultAlgaeIntake. */
  public DefaultAlgaeIntakeCommand(AlgaeIntake algaeIntake, Function<String, Object> commandData) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeIntake = algaeIntake;
    this.commandData = commandData;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Object speed = commandData.apply(Constants.MapConstants.INTAKE_SPEED);
    if (speed != null) {
      algaeIntake.runIntake((double) speed);
    } 
    else {
      algaeIntake.stopIntake();
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
