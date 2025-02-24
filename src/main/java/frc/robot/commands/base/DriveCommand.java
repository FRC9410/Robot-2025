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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utils.Utils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private final Vision vision;
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private HolonomicDriveController holonomicController;
  private PIDController strafePidController;
  private PIDController forwardPidController;

  /** Creates a new DriveCommand. */
  public DriveCommand(CommandSwerveDrivetrain drivetrain,
    CommandXboxController controller,
    Vision vision) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.vision = vision;
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Drive Command");
    this.holonomicController = new HolonomicDriveController(
      new PIDController(1, 0, 0),
      new PIDController(1.0, 0, 0),
      new ProfiledPIDController(7.0, 0, 0.0,
            new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE*2, drivetrain.MAX_ANGULAR_RATE*4)));
    this.strafePidController = new PIDController(0.01, 0, 0.0001);
    this.forwardPidController = new PIDController(0.04, 0, 0);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d pose = drivetrain.getState().Pose;
    table.getEntry("Heading").setDouble(pose.getRotation().getDegrees());
    table.getEntry("X").setDouble(pose.getTranslation().getX());
    table.getEntry("Y").setDouble(pose.getTranslation().getY());
    final int tagId = vision.getReefTag();
    final List<Integer> reefTagIds = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    if ((controller.getLeftTriggerAxis() > 0.5
      || controller.getRightTriggerAxis() > 0.5)
      && reefTagIds.contains(tagId)) {
        drivetrain.setControl(drivetrain.ROBOT_RELATIVE
        // .withVelocityX(-getForward() * drivetrain.MAX_SPEED)
          .withVelocityX(getForward() * drivetrain.MAX_SPEED)
          .withVelocityY(getStrafe() * drivetrain.MAX_SPEED)
          .withRotationalRate(getRotation()));
      } else {
        drivetrain.setControl(drivetrain.FIELD_RELATIVE
          .withVelocityX(Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED)
          .withVelocityY(Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED)
          .withRotationalRate(getRotation()));
      }

    // drivetrain.drive(
    //   controller.getLeftY() * drivetrain.MAX_SPEED,
    //   controller.getLeftX() * drivetrain.MAX_SPEED,
    //   getRotation(),
    //   CommandSwerveDrivetrain.DriveMode.FIELD_RELATIVE
    // );
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
    if (controller.leftBumper().getAsBoolean()) {
      return holonomicController.calculate(
        drivetrain.getState().Pose,
        drivetrain.getState().Pose,
        0.0,
        Rotation2d.fromDegrees(125.0)
      ).omegaRadiansPerSecond;
    } else if (controller.rightBumper().getAsBoolean()) {
      return holonomicController.calculate(
        drivetrain.getState().Pose,
        drivetrain.getState().Pose,
        0.0,
        Rotation2d.fromDegrees(-125.0)
      ).omegaRadiansPerSecond;
    } else if (controller.getLeftTriggerAxis() > 0.5
      || controller.getRightTriggerAxis() > 0.5) {
        final double scoringAngle = getScoringAngle();
        if (scoringAngle != -1.0) {
          return holonomicController.calculate(
            drivetrain.getState().Pose,
            drivetrain.getState().Pose,
            0.0,
            Rotation2d.fromDegrees(getScoringAngle())
          ).omegaRadiansPerSecond;
      }
    }
    return -Utils.squareInput(controller.getRightX()) * drivetrain.MAX_ANGULAR_RATE;
  }

  private double getStrafe() {
    if (controller.getLeftTriggerAxis() > 0.5
      || controller.getRightTriggerAxis() > 0.5) {
        final int tagId = vision.getReefTag();
        final List<Integer> reefTagIds = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        if (reefTagIds.contains(tagId)) {
          if (controller.getLeftTriggerAxis() > 0.5) {
            return strafePidController.calculate((vision.getXOffset(vision.getRightReefTable()) + 10.0));
          } else {
            return strafePidController.calculate((vision.getXOffset(vision.getLeftReefTable()) - 10.0));
          }
        }
      }
    return Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED;
  }

  private double getForward() {
    if (controller.getLeftTriggerAxis() > 0.5
    || controller.getRightTriggerAxis() > 0.5) {
      final int tagId = vision.getReefTag();
      final List<Integer> reefTagIds = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
      if (reefTagIds.contains(tagId)) {
        if (controller.getLeftTriggerAxis() > 0.5) {
          return -forwardPidController.calculate((5.0 - vision.getArea(vision.getLeftReefTable())));
        } else {
          return -forwardPidController.calculate((5.0 - vision.getArea(vision.getRightReefTable())));
        }
      } else {
        return Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED;
      }
    } else {
      return Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED;
    }
  }

  private double getScoringAngle() {
    final int tagId = vision.getReefTag();
    final List<Integer> reefTagIds = Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    if (reefTagIds.contains(tagId)) {
      if (tagId == 6 || tagId == 19) { // front left
        return 120;
      } else if (tagId == 7 || tagId == 18) { // front center
        return 179.9;
      } else if (tagId == 8 || tagId == 17) { //front right
        return -120.0;
      } else if (tagId == 9 || tagId == 22) { //back right
        return -60.0;
      } else if (tagId == 10 || tagId == 21) { //back center
        return 0.0;
      } else { // back left
        return 60.0;
      }
    } else {
      return -1.0;
    }
  }
}
