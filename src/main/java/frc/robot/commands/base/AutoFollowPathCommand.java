// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoFollowPathCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private final double x;
  private final double y;
  private final double theta;
  private Command pathFollowCommand;

  /** Creates a new FollowPathCommand. */
  public AutoFollowPathCommand(
    CommandSwerveDrivetrain drivetrain,
    CommandXboxController controller,
    double x,
    double y,
    double theta) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.x = x;
    this.y = y;
    this.theta = theta;

    pathFollowCommand = findPath();
    // pathFollowCommand = AutoBuilder.followPath(getAutonomousPath(drivetrain.getState().Pose));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathFollowCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing FollowPathCommand");
    if (Math.abs(controller.getLeftX()) > 0.5 || Math.abs(controller.getLeftY()) > 0.5 || Math.abs(controller.getRightX()) > 0.5) {
      pathFollowCommand.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending FollowPathCommand");
    if (pathFollowCommand.isScheduled()) {
      pathFollowCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !pathFollowCommand.isScheduled();
  }

  public PathPlannerPath getAutonomousPath(Pose2d currentPose) {// Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      currentPose,
      new Pose2d(x, y, Rotation2d.fromDegrees(theta))
    );

    PathConstraints constraints = new PathConstraints(
      5.4,
      10.8,
      RotationsPerSecond.of(0.75).in(RadiansPerSecond),
      Math.pow(RotationsPerSecond.of(0.75).in(RadiansPerSecond),2));

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.0, Rotation2d.fromDegrees(theta)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    System.out.println("Path created: " + path);
    return path;
  }

  public Command findPath() {// Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(theta));
    
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      3,
      4,
      Units.degreesToRadians(270),
      Units.degreesToRadians(540));
    
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
      targetPose,
      constraints,
      0.0
    );
  }
}
