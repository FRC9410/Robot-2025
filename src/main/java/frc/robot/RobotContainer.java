// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.defaults.DefaultAlgaeIntakeCommand;
import frc.robot.commands.defaults.DefaultAlgaeWristCommand;
import frc.robot.commands.defaults.DefaultClimberCommand;
import frc.robot.commands.defaults.DefaultElevatorCommand;
import frc.robot.commands.defaults.DefaultEndEffectorCommand;
import frc.robot.commands.defaults.DefaultHopperCommand;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private final Subsystems subsystems;
  public RobotContainer() {
    subsystems = new Subsystems();
    subsystems.getHopper().setDefaultCommand(new DefaultHopperCommand(subsystems.getHopper(), (key) -> subsystems.getCommandData(key)));
    subsystems.getAlgaeIntake().setDefaultCommand(new DefaultAlgaeIntakeCommand(subsystems.getAlgaeIntake(), (key) -> subsystems.getCommandData(key)));
    subsystems.getAlgaeWrist().setDefaultCommand(new DefaultAlgaeWristCommand(subsystems.getAlgaeWrist(), (key) -> subsystems.getCommandData(key)));
    subsystems.getElevator().setDefaultCommand(new DefaultElevatorCommand(subsystems.getElevator(), (key) -> subsystems.getCommandData(key)));
    subsystems.getEndEffector().setDefaultCommand(new DefaultEndEffectorCommand(subsystems.getEndEffector(), (key) -> subsystems.getCommandData(key)));
    subsystems.getClimber().setDefaultCommand(new DefaultClimberCommand(subsystems.getClimber(), (key) -> subsystems.getCommandData(key)));
    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
