// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  private final Climber climber;
  private final double direction;
  /** Creates a new DefaultClimber. */
  public ClimberCommand(Climber climber, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.direction = direction;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setVoltage(1 * direction);
    System.out.println("cimbing");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setVoltage(Constants.ClimberConstants.STOP_VOLTAGE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
