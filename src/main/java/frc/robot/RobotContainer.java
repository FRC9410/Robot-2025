// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.action.*;
import frc.robot.commands.base.*;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.ActionController.Action;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.Utils;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final Subsystems subsystems;
  private final CommandXboxController driverController;
  // private final CommandXboxController copilotController;
  // private final CommandXboxController testController;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    subsystems = new Subsystems();
    // subsystems.getHopper().setDefaultCommand(new ActionHopperCommand(subsystems.getHopper(), subsystems.getActionController()));
    // subsystems.getAlgaeIntake().setDefaultCommand(new ActionAlgaeIntakeCommand(subsystems.getAlgaeIntake(), subsystems.getActionController()));
    // subsystems.getAlgaeWrist().setDefaultCommand(new ActionAlgaeWristCommand(subsystems.getAlgaeWrist(), subsystems.getActionController()));
    // subsystems.getElevator().setDefaultCommand(new ActionElevatorCommand(subsystems.getElevator(), subsystems.getActionController()));
    // subsystems.getEndEffector().setDefaultCommand(new ActionEndEffectorCommand(subsystems.getEndEffector(), subsystems.getActionController()));
    // subsystems.getClimber().setDefaultCommand(new ActionClimberCommand(subsystems.getClimber(), subsystems.getActionController()));
    // subsystems.getActionController().setDefaultCommand(new ActionRequestCommand(subsystems, Action.IDLE));

    driverController = new CommandXboxController(0);
    // copilotController = new CommandXboxController(1);
    // testController = new CommandXboxController(2);
    
    configurePilotBindings();
    // configureCopilotBindings();
    // configureTestBindings();
  }

  private void configurePilotBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    driverController.back().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION));
    driverController.a().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L1_SCORE_POSITION));
    driverController.b().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L2_SCORE_POSITION));
    driverController.x().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L3_SCORE_POSITION));
    driverController.y().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION));
    driverController.rightBumper().onTrue(new HopperCommand(subsystems.getHopper(),subsystems.getEndEffector(), subsystems.getSensors(), Constants.HopperConstants.START_VOLTAGE, Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE));
    driverController.rightTrigger(0.5).whileTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // private void configureCopilotBindings() {
  //   copilotController.start().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION));
  //   copilotController.a().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L1_SCORE_POSITION));
  //   copilotController.b().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L2_SCORE_POSITION));
  //   copilotController.x().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L3_SCORE_POSITION));
  //   copilotController.y().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION));
  //   copilotController.rightBumper().onTrue(new HopperCommand(subsystems.getHopper(),subsystems.getEndEffector(), subsystems.getSensors(), Constants.HopperConstants.START_VOLTAGE, Constants.EndEffectorConstants.END_EFFECTOR_INTAKE_VOLTAGE));
  //   copilotController.rightTrigger(0.5).whileTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE));
  // }

  // private void configureTestBindings() {
  //   // testController.povUp().whileTrue(new ElevatorCommand(subsystems.getElevator(), Constants.ElevatorConstants.UP_VOLTAGE));
  //   // testController.povDown().whileTrue(new ElevatorCommand(subsystems.getElevator(), Constants.ElevatorConstants.DOWN_VOLTAGE));
  //   // testController.a().whileTrue(new HopperCommand(subsystems.getHopper(), Constants.HopperConstants.START_VOLTAGE));
  //   testController.x().whileTrue(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE));
  //   // testController.b().whileTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), Constants.AlgaeWristConstants.WRIST_DOWN_VOLTAGE));
  //   testController.y().whileTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.23));
  //   testController.rightBumper().whileTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE));
  //   testController.start().whileTrue(new ClimberCommand(subsystems.getClimber(), Constants.ClimberConstants.CLIMB_VOLTAGE));
  //   testController.back().whileTrue(new ClimberCommand(subsystems.getClimber(), -Constants.ClimberConstants.CLIMB_VOLTAGE));
  // }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void updatePose() {
    final PoseEstimate poseEsimate = subsystems.getVision().getPose(true);
    
        // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        // m_poseEstimator.addVisionMeasurement(
        //     mt2.pose,
        //     mt2.timestampSeconds);
  }

  public void logMapData() {
    Utils.logMap(subsystems.getSubsystemData(), "Subsystem Data");
    Utils.logMap(subsystems.getActionController().getCommandData(), "Command Data");
  }
}
