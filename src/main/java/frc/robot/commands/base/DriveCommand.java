// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

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

      table.getEntry("X P").setDouble(0.8);
      table.getEntry("X D").setDouble(0.05);
      table.getEntry("Y P").setDouble(0.8);
      table.getEntry("Y D").setDouble(0.05);
      table.getEntry("Rotation P").setDouble(1.2);
      table.getEntry("Rotation D").setDouble(0.05);

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
      final Pose2d targetPose = (Pose2d) actionController.getCommandField(Constants.MapConstants.TARGET_POSE);
      final ChassisSpeeds ChassisSpeeds = holonomicController.calculate(
        currentPose,
        targetPose,
        0.0,
        targetPose.getRotation()
      );
      table.getEntry("Y Delta").setDouble(targetPose.getTranslation().getX() - currentPose.getTranslation().getX());
      table.getEntry("X Delta").setDouble(targetPose.getTranslation().getY() - currentPose.getTranslation().getY());
      table.getEntry("Rotation Delta").setDouble(targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees());

      final double xSpeed = ChassisSpeeds.vxMetersPerSecond * drivetrain.MAX_SPEED;
      final double xSpeedDirection = xSpeed > 1.0 ? 1 : -1;
      final double ySpeed = ChassisSpeeds.vyMetersPerSecond * drivetrain.MAX_SPEED;
      final double ySpeedDirection = ySpeed > 1.0 ? 1 : -1;
      final double maxSpeed = 0.5 * drivetrain.MAX_SPEED;

      drivetrain.setControl(drivetrain.ROBOT_RELATIVE
        .withVelocityX(Math.abs(xSpeed) > maxSpeed ? maxSpeed * xSpeedDirection  : xSpeed)
        .withVelocityY(Math.abs(ySpeed) > maxSpeed ? maxSpeed * ySpeedDirection : ySpeed)
        .withRotationalRate(ChassisSpeeds.omegaRadiansPerSecond * drivetrain.MAX_ANGULAR_RATE));
    } else if (dashboard.getIsClimbing()) {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
    } else if (actionController.getCommandField(Constants.MapConstants.ELEVATOR_POSITION) != null
    && (Double) actionController.getCommandField(Constants.MapConstants.ELEVATOR_POSITION) > 0.0) {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
    } else {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.cubeInput(controller.getLeftY()) * drivetrain.MAX_SPEED)
        .withVelocityY(direction * Utils.cubeInput(controller.getLeftX()) * drivetrain.MAX_SPEED)
        .withRotationalRate(getRotation()));
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
