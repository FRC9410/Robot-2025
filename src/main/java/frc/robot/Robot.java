// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    CanBridge.runTCP();
    robotContainer.resetLocalization();
    // robotContainer.getSubsystems().getDrivetrain().configureAutoBuilder();
    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320,240,30);
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.updatePose();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = new WaitCommand(0.010).andThen(robotContainer.getAutonomousCommand());

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
    robotContainer.logMapData();
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
