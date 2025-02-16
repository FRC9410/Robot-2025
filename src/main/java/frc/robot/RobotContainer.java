// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.action.*;
import frc.robot.commands.base.*;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.ActionController.Action;

public class RobotContainer {
  private final Subsystems subsystems;
  private final CommandXboxController driverController;
  private final CommandXboxController copilotController;
  private final CommandXboxController testController;

  public RobotContainer() {
    subsystems = new Subsystems();
    subsystems.getHopper().setDefaultCommand(new ActionHopperCommand(subsystems.getHopper(), subsystems.getActionController()));
    subsystems.getAlgaeIntake().setDefaultCommand(new ActionAlgaeIntakeCommand(subsystems.getAlgaeIntake(), subsystems.getActionController()));
    subsystems.getAlgaeWrist().setDefaultCommand(new ActionAlgaeWristCommand(subsystems.getAlgaeWrist(), subsystems.getActionController()));
    subsystems.getElevator().setDefaultCommand(new ActionElevatorCommand(subsystems.getElevator(), subsystems.getActionController()));
    subsystems.getEndEffector().setDefaultCommand(new ActionEndEffectorCommand(subsystems.getEndEffector(), subsystems.getActionController()));
    subsystems.getClimber().setDefaultCommand(new ActionClimberCommand(subsystems.getClimber(), subsystems.getActionController()));
    subsystems.getActionController().setDefaultCommand(new ActionRequestCommand(subsystems, Action.IDLE));

    driverController = new CommandXboxController(0);
    copilotController = new CommandXboxController(1);
    testController = new CommandXboxController(2);
    
    configurePilotBindings();
    configureCopilotBindings();
    configureTestBindings();
  }

  private void configurePilotBindings() {}

  private void configureCopilotBindings() {}

  private void configureTestBindings() {
    testController.povUp().whileTrue(new ElevatorCommand(subsystems.getElevator(), Constants.ElevatorConstants.ELEVATOR_UP_VOLTAGE));
    testController.povDown().whileTrue(new ElevatorCommand(subsystems.getElevator(), Constants.ElevatorConstants.ELEVATOR_DOWN_VOLTAGE));
    testController.a().whileTrue(new HopperCommand(subsystems.getHopper(), Constants.HopperConstants.HOPPER_VOLTAGE));
    testController.x().whileTrue(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE));
    testController.b().whileTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE));
    testController.y().whileTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), Constants.AlgaeWristConstants.WRIST_UP_VOLTAGE));
    testController.rightBumper().whileTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE));
    testController.start().whileTrue(new ClimberCommand(subsystems.getClimber(), Constants.ClimberConstants.CLIMB_VOLTAGE));
    testController.back().whileTrue(new ClimberCommand(subsystems.getClimber(), -Constants.ClimberConstants.CLIMB_VOLTAGE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
