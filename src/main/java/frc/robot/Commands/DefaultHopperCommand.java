// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultHopperCommand extends Command {
  private Hopper hopper;
  private Function<String, Object> commandData;
  /** Creates a new DefaultHopper. */
  public DefaultHopperCommand(Hopper hopper, Function<String, Object> commandData) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.commandData = commandData;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object speed = commandData.apply(Constants.MapConstants.HOPPER_SPEED);
    if (speed != null) {
      // Use the speed variable as needed
      hopper.runHopper((double) speed);
    } else {
      hopper.stopHopper();
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