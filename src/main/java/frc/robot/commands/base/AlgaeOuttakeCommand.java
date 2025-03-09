// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOuttakeCommand extends Command {
  private final AlgaeIntake algaeIntake;
  private final double voltage;
  private Timer timer;
  /** Creates a new DefaultAlgaeIntake. */
  public AlgaeOuttakeCommand(AlgaeIntake algaeIntake, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeIntake = algaeIntake;
    this.voltage = voltage;
    this.timer = new Timer();

    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.timer.hasElapsed(0.5)) {
      algaeIntake.outtakeAlgae();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
