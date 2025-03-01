// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.action.ActionElevatorCommand;
import frc.robot.commands.base.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Subsystems;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class RobotContainer {
  private double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric FIELD_DRIVE = new SwerveRequest.FieldCentric()
    .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric ROBOT_DRIVE = new SwerveRequest.RobotCentric()
    .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Subsystems subsystems;
  private final CommandXboxController driverController;
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final SendableChooser<Command> autoChooser;

  // private final Telemetry logger = new Telemetry(MAX_SPEED);

  private String humanPlayerStation = "";

  public RobotContainer() {
    subsystems = new Subsystems();
    // subsystems.getHopper().setDefaultCommand(new ActionHopperCommand(subsystems.getHopper(), subsystems.getActionController()));
    // subsystems.getAlgaeIntake().setDefaultCommand(new ActionAlgaeIntakeCommand(subsystems.getAlgaeIntake(), subsystems.getActionController()));
    // subsystems.getAlgaeWrist().setDefaultCommand(new ActionAlgaeWristCommand(subsystems.getAlgaeWrist(), subsystems.getActionController()));
    subsystems.getElevator().setDefaultCommand(new ActionElevatorCommand(subsystems.getElevator(), subsystems.getSensors()));
    subsystems.getEndEffector().setDefaultCommand(new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors()));
    // subsystems.getClimber().setDefaultCommand(new ActionClimberCommand(subsystems.getClimber(), subsystems.getActionController()));
    // subsystems.getActionController().setDefaultCommand(new ActionRequestCommand(subsystems, Action.IDLE));

    driverController = new CommandXboxController(0);
    
    configurePilotBindings();
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Vision Logging");
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configurePilotBindings() {
    subsystems.getDrivetrain().setDefaultCommand(
      new DriveCommand(
        subsystems.getDrivetrain(),
        driverController, 
        subsystems.getVision()));

    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(subsystems.getDrivetrain().runOnce(() -> {
      subsystems.getDrivetrain().resetPose(new Pose2d());
    }));
    driverController.rightBumper().whileTrue(
      new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L1_SCORE_POSITION));
    driverController.rightBumper().onFalse(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors()));
    driverController.leftBumper().onTrue(
      new HopperCommand(
        subsystems.getHopper(),
        subsystems.getSensors(),
        Constants.HopperConstants.START_VOLTAGE));

    driverController.povLeft().onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L1_ALGAE_POSITION)
    .alongWith(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.40)
    // .alongWith(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE))
    ));

    driverController.povLeft().onFalse(new SequentialCommandGroup(new WaitCommand(0.25), new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION)));;

    // subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public void updatePose() {
    final String bestLimelight = subsystems.getVision().getBestLimelight();

    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(bestLimelight);
    LimelightHelpers.PoseEstimate bestMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight);
    

    if (bestMeasurement != null && bestMeasurement.avgTagArea > 0.2) {
      Pose2d newPose = pose.toPose2d();
      subsystems.getDrivetrain().resetRotation(newPose.getRotation());
    } 


    LimelightHelpers.SetRobotOrientation("limelight-lftper", subsystems.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-rghtper", subsystems.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-lftrf", subsystems.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-rghtrf", subsystems.getDrivetrain().getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
    table.getEntry("tag-count").setDouble(mt2.tagCount);
    table.getEntry("angular-velo").setBoolean(Math.abs(subsystems.getDrivetrain().getPigeon2().getRate()) < 720);

    if(mt2 != null && subsystems.getDrivetrain().getPigeon2().getRate() < 720
      && mt2.tagCount > 0) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    {
      subsystems.getDrivetrain().resetPose(mt2.pose);
      subsystems.getDrivetrain().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      subsystems.getDrivetrain().addVisionMeasurement(
          mt2.pose,
          Utils.fpgaToCurrentTime(mt2.timestampSeconds));
    }

  }

  public void logMapData() {
    frc.robot.utils.Utils.logMap(subsystems.getSubsystemData(), "Subsystem Data");
    frc.robot.utils.Utils.logMap(subsystems.getActionController().getCommandData(), "Command Data");
  }

  public void resetLocalization() {
    subsystems.getDrivetrain().resetPose(new Pose2d());
    // subsystems.getDrivetrain().seedFieldCentric();
  }

  public Subsystems getSubsystems() {
    return subsystems;
  }

  public void registerNamedCommands() {
    
  }
}
