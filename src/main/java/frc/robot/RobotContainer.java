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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.commands.action.ActionElevatorCommand;
import frc.robot.commands.action.ActionHopperCommand;
import frc.robot.commands.base.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Dashboard.Auto;
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
  private final CommandXboxController testController;
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final SendableChooser<Command> autoChooser;

  // private final Telemetry logger = new Telemetry(MAX_SPEED);

  public RobotContainer() {
    subsystems = new Subsystems();
    subsystems.getHopper().setDefaultCommand(new ActionHopperCommand(subsystems.getHopper(), subsystems.getActionController(), subsystems.getSensors()));
    subsystems.getElevator().setDefaultCommand(new ActionElevatorCommand(subsystems.getElevator(), subsystems.getActionController()));
    // subsystems.getAlgaeIntake().setDefaultCommand(new ActionAlgaeIntakeCommand(subsystems.getAlgaeIntake(), subsystems.getActionController()));
    // subsystems.getAlgaeWrist().setDefaultCommand(new ActionAlgaeWristCommand(subsystems.getAlgaeWrist(), subsystems.getActionController()));
    subsystems.getEndEffector().setDefaultCommand(new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()));
    // subsystems.getClimber().setDefaultCommand(new ActionClimberCommand(subsystems.getClimber(), subsystems.getActionController()));
    // subsystems.getActionController().setDefaultCommand(new ActionRequestCommand(subsystems, Action.IDLE));

    driverController = new CommandXboxController(0);
    testController = new CommandXboxController(5);
    
    configurePilotBindings();
    configureTestBindings();
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Vision Logging");
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configurePilotBindings() {
    subsystems.getDrivetrain().setDefaultCommand(
      new ProfiledDriveCommand(
        subsystems.getDrivetrain(),
        driverController, 
        subsystems.getDashboard(),
        subsystems.getActionController(),
        subsystems.getElevator()));

    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(subsystems.getDrivetrain().runOnce(() -> {
      subsystems.getDrivetrain().resetPose(new Pose2d());
    }));

    // driverController.start().onTrue(subsystems.getActionController().runOnce(() -> {
    //   subsystems.getActionController().toggleAutoMode();
    // }));

    driverController.rightTrigger(0.5).and(driverController.leftTrigger(0.5).negate()).onTrue(subsystems.getActionController().runOnce(() -> {
      subsystems.getActionController().setAutoMode(true);
    }));

    driverController.rightTrigger(0.5).and(driverController.leftTrigger(0.5).negate()).onFalse(subsystems.getActionController().runOnce(() -> {
      subsystems.getActionController().setAutoMode(false);
    }));


    driverController.b().and(driverController.leftTrigger(0.5).negate()).onTrue(new ElevatorCoralPositionCommand(subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), false));



    // driverController.rightTrigger(0.5).and(driverController.leftTrigger(0.5).negate()).onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard()));
    driverController.rightBumper().and(driverController.leftTrigger(0.5).negate()).whileTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController()));
    driverController.x().and(driverController.leftTrigger(0.5).negate()).whileTrue(new HopperCommand(subsystems.getHopper(), subsystems.getSensors()));
    driverController.a().onTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION, false));



    // driverController.rightTrigger(0.5).and(driverController.leftTrigger(0.5)).onTrue(new ElevatorCommand(subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard()));
    // driverController.rightBumper().and(driverController.leftTrigger(0.5)).onTrue(new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController()));
    // driverController.povRight().and(driverController.leftTrigger(0.5)).onTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION));

    
    driverController.leftBumper().and(driverController.leftTrigger(0.5)).onTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.37, subsystems.getAlgaeIntake())
      .alongWith(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE)));
    
      driverController.leftBumper().and(driverController.leftTrigger(0.5)).onFalse(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.07, subsystems.getAlgaeIntake())
        .alongWith(new SequentialCommandGroup(
          new WaitCommand(0.25),
          new AlgaeStopIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.STOP_VOLTAGE)
        )));
      
    driverController.x().and(driverController.leftTrigger(0.5)).whileTrue(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.33, subsystems.getAlgaeIntake())
    .alongWith(new AlgaeOuttakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE)));
    
    driverController.rightTrigger(0.5).and(driverController.leftTrigger(0.5)).whileTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, true)
      .alongWith(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.13, subsystems.getAlgaeIntake())
      .alongWith(new SequentialCommandGroup(
        new WaitCommand(0.25),
        new AlgaeOuttakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE)
      ))));

      driverController.rightBumper().and(driverController.leftTrigger(0.5)).onTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L1_ALGAE_POSITION, true)
        .alongWith(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.31, subsystems.getAlgaeIntake())
        .alongWith(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE))));
  
      driverController.rightBumper().and(driverController.leftTrigger(0.5)).onFalse(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION, true)
      .alongWith(new SequentialCommandGroup(
        new WaitCommand(0.0),
        new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.05, subsystems.getAlgaeIntake()))
      .alongWith(new AlgaeStopIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE))));

      driverController.b().and(driverController.leftTrigger(0.5)).onTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L2_ALGAE_POSITION, true)
        .alongWith(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.31, subsystems.getAlgaeIntake())
        .alongWith(new AlgaeOuttakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE))));
  
      driverController.b().and(driverController.leftTrigger(0.5)).onFalse(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION, true)
      .alongWith(new SequentialCommandGroup(
        new WaitCommand(0.25),
        new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.05
        
        , subsystems.getAlgaeIntake()))
      .alongWith(new AlgaeStopIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE))));





    // driverController.povLeft().onTrue(new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L2_ALGAE_POSITION)
    // .alongWith(new AlgaeWristCommand(subsystems.getAlgaeWrist(), 0.35)
    // ));

    // driverController.povLeft().whileTrue(new AlgaeIntakeCommand(subsystems.getAlgaeIntake(), Constants.AlgaeIntakeConstants.INTAKE_VOLTAGE));

    // driverController.povLeft().onFalse(new SequentialCommandGroup(new WaitCommand(0.25), new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.HOME_POSITION)));

    driverController.y().whileTrue(new ClimberCommand(subsystems.getClimber(), 1));

    // subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);
  }

  
  private void configureTestBindings() {
    testController.y().whileTrue(new ClimberCommand(subsystems.getClimber(), 1));
    testController.x().whileTrue(new ClimberCommand(subsystems.getClimber(), -1));
  }

  public Command getAutonomousCommand() {
    Auto selectedAuto = (Auto) subsystems.getDashboard().getAuto();

    switch (selectedAuto) {
      case RED_LEFT:
        return getRedLeftCommand();
      case RED_RIGHT:
        return getRedRightCommand();
      case BLUE_LEFT:
        return getBlueLeftCommand();
      case BLUE_RIGHT:
        return getBlueRightCommand();
      default:
        return new WaitCommand(1);
    }
  }

  public void updatePose() {
    final String bestLimelight = subsystems.getVision().getBestLimelight();

    if (bestLimelight.isEmpty()) {
      return;
    }

    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(bestLimelight);
    LimelightHelpers.PoseEstimate bestMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(bestLimelight);
    

    if (bestMeasurement != null && bestMeasurement.avgTagArea > 0.1) {
      Pose2d newPose = pose.toPose2d();
      subsystems.getDrivetrain().resetRotation(newPose.getRotation());
      LimelightHelpers.SetRobotOrientation("limelight-left", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-right", newPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }  else {
      return;
    }

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

  public Command getRedLeftCommand() {
    return new SequentialCommandGroup(
        new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_BACK_LEFT_RIGHT),
        new ParallelRaceGroup(
          new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
          new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
        ),
        new GotoDriveCommand(subsystems.getDrivetrain(), new Pose2d(12.536, 1.715, Rotation2d.fromDegrees(125.0))),
        new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_HP_LEFT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
        new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
        new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_FRONT_LEFT_LEFT)),
        new ParallelRaceGroup(
          new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
          new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
        ),
        new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
        new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_HP_LEFT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
        new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
        new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_FRONT_LEFT_RIGHT)),
        new ParallelRaceGroup(
          new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
          new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
        )
      );
  }

  public Command getRedRightCommand() {
    return new SequentialCommandGroup(
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_BACK_RIGHT_LEFT),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), new Pose2d(12.536, 6.435, Rotation2d.fromDegrees(-125.0))),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_HP_RIGHT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_FRONT_RIGHT_LEFT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_HP_RIGHT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.RED_FRONT_RIGHT_RIGHT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      )
    );
  }

  public Command getBlueLeftCommand() {
      return new SequentialCommandGroup(
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_BACK_LEFT_RIGHT),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), new Pose2d(5.304, 6.135, Rotation2d.fromDegrees(-55.0))),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_HP_LEFT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_FRONT_LEFT_LEFT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_HP_LEFT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_FRONT_LEFT_RIGHT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      )
    );
  }

  public Command getBlueRightCommand() {
    return new SequentialCommandGroup(
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_BACK_RIGHT_LEFT),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), new Pose2d(5.014, 1.815, Rotation2d.fromDegrees(55.0))),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_HP_RIGHT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_FRONT_RIGHT_LEFT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      ),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_HP_RIGHT),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new WaitCommand(1)),
      new ParallelRaceGroup(
      new DefaultEndEffectorCommand(subsystems.getEndEffector(), subsystems.getSensors(), subsystems.getElevator(), subsystems.getDashboard()),
      new HopperCommand(subsystems.getHopper(), subsystems.getSensors()),
      new GotoDriveCommand(subsystems.getDrivetrain(), ScoringConstants.BLUE_FRONT_RIGHT_RIGHT)),
      new ParallelRaceGroup(
        new ElevatorPositionCommand(subsystems.getElevator(), subsystems.getSensors(), Constants.ElevatorConstants.L4_SCORE_POSITION, false),
        new EndEffectorCommand(subsystems.getEndEffector(), Constants.EndEffectorConstants.END_EFFECTOR_VOLTAGE, subsystems.getElevator(), subsystems.getSensors(), subsystems.getDashboard(), subsystems.getActionController())
      )
    );
  }
}
