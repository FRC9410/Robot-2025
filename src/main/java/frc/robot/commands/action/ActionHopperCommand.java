// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.action;


import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActionHopperCommand extends Command {
  private final Hopper hopper;
  private final ActionController controller;
  private final Sensors sensors;
  /** Creates a new DefaultHopper. */
  public ActionHopperCommand(Hopper hopper, ActionController controller, Sensors sensors) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.controller = controller;
    this.sensors = sensors;

    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Object voltage = controller.getCommandField(Constants.MapConstants.HOPPER_VOLTAGE);
    if (voltage != null) {
      // Use the speed variable as needed
      hopper.setVoltage((double) voltage);
    // } else if (!sensors.getIntakeLaserBroken()) {
    //   hopper.setVoltage((double) HopperConstants.START_VOLTAGE);
    } else {
      hopper.setVoltage(Constants.HopperConstants.STOP_VOLTAGE);
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