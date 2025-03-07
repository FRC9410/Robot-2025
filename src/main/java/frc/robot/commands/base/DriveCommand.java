// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Dashboard.CoralSide;
import frc.robot.utils.Utils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private HolonomicDriveController holonomicController;
  private double direction;
  private Dashboard dashboard;
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final ActionController actionController;

  /** Creates a new DriveCommand. */
  public DriveCommand(CommandSwerveDrivetrain drivetrain,
    CommandXboxController controller,
    Dashboard dashboard,
    ActionController actionController) {
      inst = NetworkTableInstance.getDefault();
      table = inst.getTable("Driving PIDs");

      this.drivetrain = drivetrain;
      this.controller = controller;
      this.dashboard = dashboard;
      this.actionController = actionController;
      this.holonomicController = new HolonomicDriveController(
        new PIDController(0.08, 0, 0.0002),
        new PIDController(0.08, 0, 0),
        new ProfiledPIDController(0.06, 0, 0.0,
              new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE*2, drivetrain.MAX_ANGULAR_RATE*4)));
      direction = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1.0 : 1.0;

      table.getEntry("X P").setDouble(0.45);
      table.getEntry("X D").setDouble(0.00015);
      table.getEntry("Y P").setDouble(0.45);
      table.getEntry("Y D").setDouble(0.00015);
      table.getEntry("Rotation P").setDouble(0.8);
      table.getEntry("Rotation D").setDouble(0.00015);

      addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    setRotationPid();
    final Pose2d currentPose = drivetrain.getState().Pose;
    System.out.println(actionController.getCommandField(Constants.MapConstants.TARGET_POSE));
    if (currentPose != null
    && actionController.getCommandField(Constants.MapConstants.TARGET_POSE) != null
    && Math.abs(controller.getLeftX()) < 0.10
    && Math.abs(controller.getLeftY()) < 0.10
    && Math.abs(controller.getRightX()) < 0.10) {
      final Pose2d requestedPose = (Pose2d) actionController.getCommandField(Constants.MapConstants.TARGET_POSE);
      final Pose2d targetPose = Utils.pathIntersectsHexagon(new Point2D.Double(currentPose.getX(), currentPose.getY()), new Point2D.Double(requestedPose.getX(), requestedPose.getY())) ? Utils.findSafeWaypoint(currentPose, requestedPose) : requestedPose;
      final ChassisSpeeds ChassisSpeeds = holonomicController.calculate(
        currentPose,
        targetPose,
        0.0,
        targetPose.getRotation()
      );
      final double xDelta = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
      final double yDelta = targetPose.getTranslation().getY() - currentPose.getTranslation().getY();
      final double rotationDelta = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
      table.getEntry("Y Delta").setDouble(xDelta);
      table.getEntry("X Delta").setDouble(yDelta);
      table.getEntry("Rotation Delta").setDouble(rotationDelta);

      final boolean isWithinTolerance = Math.abs(xDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE &&
        Math.abs(yDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE
        && Math.abs(rotationDelta) < Constants.AutoConstants.ROTATION_TOLERANCE;

      if (isWithinTolerance) {
        final double xChassisSpeed = ChassisSpeeds.vxMetersPerSecond * drivetrain.MAX_SPEED;
        final double yChassisSpeed = ChassisSpeeds.vyMetersPerSecond * drivetrain.MAX_SPEED;
        final double chassisTurnSpeed = ChassisSpeeds.omegaRadiansPerSecond * drivetrain.MAX_ANGULAR_RATE;

        final double xSpeedDirection = xChassisSpeed > 1.0 ? 1 : -1;
        final double ySpeedDirection = yChassisSpeed > 1.0 ? 1 : -1;
        final double turnSpeedDirection = chassisTurnSpeed > 1.0 ? 1 : -1;

        final double maxSpeed = 0.5 * drivetrain.MAX_SPEED;
        final double minSpeed = 0.1 * drivetrain.MAX_SPEED;

        final double maxTurnSpeed = 0.5 * drivetrain.MAX_ANGULAR_RATE;
        final double minTurnSpeed = 0.1 * drivetrain.MAX_ANGULAR_RATE;

        final double xSpeed = Math.abs(xChassisSpeed) < maxSpeed && Math.abs(xChassisSpeed) > minSpeed
          ? xChassisSpeed
          : Math.abs(xChassisSpeed) > maxSpeed
          ? maxSpeed * xSpeedDirection
          : minSpeed * xSpeedDirection;

        final double ySpeed = Math.abs(yChassisSpeed) < maxSpeed && Math.abs(yChassisSpeed) > minSpeed
          ? yChassisSpeed
          : Math.abs(yChassisSpeed) > maxSpeed
          ? maxSpeed * ySpeedDirection
          : minSpeed * ySpeedDirection;

        final double turnSpeed = Math.abs(chassisTurnSpeed) < maxTurnSpeed && Math.abs(chassisTurnSpeed) > minTurnSpeed
          ? chassisTurnSpeed
          : Math.abs(chassisTurnSpeed) > maxTurnSpeed
          ? maxTurnSpeed * turnSpeedDirection
          : minTurnSpeed * turnSpeedDirection;

        drivetrain.setControl(drivetrain.ROBOT_RELATIVE
          .withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(turnSpeed));
      }
    } else if (dashboard.getIsClimbing()) {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
        actionController.toggleAutoMode();
    } else if (actionController.getCommandField(Constants.MapConstants.ELEVATOR_POSITION) != null
    && (Double) actionController.getCommandField(Constants.MapConstants.ELEVATOR_POSITION) > 0.0) {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
        actionController.toggleAutoMode();
    } else {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.cubeInput(controller.getLeftY()) * drivetrain.MAX_SPEED)
        .withVelocityY(direction * Utils.cubeInput(controller.getLeftX()) * drivetrain.MAX_SPEED)
        .withRotationalRate(getRotation()));
        actionController.toggleAutoMode();
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

  private double getRotation() {
    // final Pose2d currentPose = drivetrain.getState().Pose;
    // final Pose2d targetPose = dashboard.getScoringPose();
    // if (controller.leftBumper().getAsBoolean()) {
    //   return holonomicController.calculate(
    //     drivetrain.getState().Pose,
    //     drivetrain.getState().Pose,
    //     0.0,
    //     Rotation2d.fromDegrees(125.0)
    //   ).omegaRadiansPerSecond;
    // } else {
    // }


    return -Utils.squareInput(controller.getRightX()) * drivetrain.MAX_ANGULAR_RATE;
  }

  public void setRotationPid () {
    holonomicController.getXController().setP(table.getEntry("X P").getDouble(0.006));
    holonomicController.getXController().setD(table.getEntry("X D").getDouble(0));
    holonomicController.getYController().setP(table.getEntry("Y P").getDouble(0.006));
    holonomicController.getYController().setD(table.getEntry("Y D").getDouble(0));
    holonomicController.getThetaController().setP(table.getEntry("Rotation P").getDouble(0.006));
    holonomicController.getThetaController().setD(table.getEntry("Rotation D").getDouble(0));
  }
}
