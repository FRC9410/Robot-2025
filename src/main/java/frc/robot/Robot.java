// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Subsystems;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final NetworkTableInstance inst;
  private final NetworkTable table;

  public Robot() {
    robotContainer = new RobotContainer();
    CanBridge.runTCP();
    robotContainer.resetLocalization();
    // robotContainer.getSubsystems().getDrivetrain().configureAutoBuilder();
    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320,240,30);
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Drive Command");
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();
    robotContainer.getSubsystems().getLeds().setFadeAnimtation(0, 255, 255);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(Math.pow(RotationsPerSecond.of(0.75).in(RadiansPerSecond),2));
    // System.out.println(RotationsPerSecond.of(0.75).in(RadiansPerSecond));

    robotContainer.updatePose();
    final Pose2d pose = robotContainer.getSubsystems().getDrivetrain().getState().Pose;
    table.getEntry("Heading").setDouble(pose.getRotation().getDegrees());
    table.getEntry("X").setDouble(pose.getTranslation().getX());
    table.getEntry("Y").setDouble(pose.getTranslation().getY());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = new WaitCommand(0.010).andThen(robotContainer.getRedLeftCommand());

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // robotContainer.logMapData();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
