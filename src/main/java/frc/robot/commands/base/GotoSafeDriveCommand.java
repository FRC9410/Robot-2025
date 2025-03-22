// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GotoSafeDriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final NetworkTableInstance inst;
  private final NetworkTable table;



  ///////////////////////////////////////////////////////////////////////////////////////
  //
  //  Profiled Motion Testing
  //
  ///////////////////////////////////////////////////////////////////////////////////////
  
  private final HolonomicDriveController holonomicController;
  private final PIDController TRANS_CONTROLLER;

  private final ProfiledPIDController thetaController;
  private double transFF = 0;
  private double rotationFF = 0;
  
  public final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);
  public final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

  public final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(1);

  private final Pose2d targetPose;

  ///////////////////////////////////////////////////////////////////////////////////////
  //
  //  Profiled Motion Testing END
  //
  ///////////////////////////////////////////////////////////////////////////////////////

  /** Creates a new DriveCommand. */
  public GotoSafeDriveCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
      this.targetPose = targetPose;
      this.drivetrain = drivetrain;
      inst = NetworkTableInstance.getDefault();
      table = inst.getTable("Driving Command");

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      //
      //

        thetaController = new ProfiledPIDController(
          0.7, 0, 0, new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE,
              Math.pow(drivetrain.MAX_ANGULAR_RATE, 2)));

              
        TRANS_CONTROLLER = new PIDController(
          0.5,
          0,
          0);
        
        TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

        thetaController.enableContinuousInput(-180, 180);
        thetaController.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));

        holonomicController = new HolonomicDriveController(
          TRANS_CONTROLLER,
          TRANS_CONTROLLER,
          thetaController);                // Rotation ProfiledPID

        addRequirements(drivetrain);
      //
      //
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;

    final double xDelta = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
    final double yDelta = targetPose.getTranslation().getY() - currentPose.getTranslation().getY();
    final double rotationDelta = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

    final boolean isWithinTolerance = Math.abs(xDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE &&
      Math.abs(yDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE
      && Math.abs(rotationDelta) < Constants.AutoConstants.ROTATION_TOLERANCE;

    if (!isWithinTolerance) {
      final ChassisSpeeds chassisSpeeds = holonomicController.calculate(
      currentPose,
      targetPose,
      0.0,
      targetPose.getRotation());
      
      double xChassisSpeed = chassisSpeeds.vxMetersPerSecond * drivetrain.MAX_SPEED;
      double yChassisSpeed = chassisSpeeds.vyMetersPerSecond * drivetrain.MAX_SPEED;
      double chassisTurnSpeed = chassisSpeeds.omegaRadiansPerSecond * drivetrain.MAX_ANGULAR_RATE;

      double xSpeedDirection = xChassisSpeed > 0 ? 1 : -1;
      double ySpeedDirection = yChassisSpeed > 0 ? 1 : -1;
      double turnSpeedDirection = chassisTurnSpeed > 0 ? 1 : -1;

      double transFF = table.getEntry("TransFF").getDouble(0.0); //0.25;
      double turnFF = table.getEntry("RotationFF").getDouble(0.0); //0.3;
      double totalChassisSpeed = Math.abs(xChassisSpeed) + Math.abs(yChassisSpeed);

      double xSpeed = (xChassisSpeed + (transFF * xSpeedDirection));
      double ySpeed = (yChassisSpeed + (transFF * ySpeedDirection));

      double turnSpeed = (chassisTurnSpeed + (turnFF * turnSpeedDirection));

      
      table.getEntry("X Speed").setDouble(xSpeed);
      table.getEntry("Y Speed").setDouble(ySpeed);
      table.getEntry("turT Speed").setDouble(turnSpeed);
      table.getEntry("transFFXSpeed").setDouble(transFF * xSpeedDirection);
      table.getEntry("transFFYSpeed").setDouble(transFF * ySpeedDirection);
      table.getEntry("rotationFFSpeed").setDouble(turnFF * turnSpeedDirection);
      table.getEntry("chassisXSpeed").setDouble(xChassisSpeed);
      table.getEntry("chassisYSpeed").setDouble(yChassisSpeed);
      table.getEntry("chassisTurnSpeed").setDouble(turnSpeed);

      drivetrain.setControl(drivetrain.ROBOT_RELATIVE
        .withVelocityX(Math.min(xSpeed, drivetrain.MAX_SPEED * 1.0))
        .withVelocityY(Math.min(ySpeed, drivetrain.MAX_SPEED * 1.0))
        .withRotationalRate(Math.min(turnSpeed, drivetrain.MAX_ANGULAR_RATE * 0.5)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drivetrain.ROBOT_RELATIVE.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    final double xDelta = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
    final double yDelta = targetPose.getTranslation().getY() - currentPose.getTranslation().getY();
    final double rotationDelta = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

    final boolean isWithinTolerance = Math.abs(xDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE + 1 &&
      Math.abs(yDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE + 1
      && Math.abs(rotationDelta) < Constants.AutoConstants.ROTATION_TOLERANCE + 10;

    if (isWithinTolerance) {
      return true;
    }
    return false;
  }

  // public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose, double dt) {
  //   // Generate trapezoidal profiles for X and Y
  //   TrapezoidProfile xProfile = new TrapezoidProfile(xConstraints);
  //   TrapezoidProfile yProfile = new TrapezoidProfile(yConstraints);

  //   // Update profile states
  //   xState = xProfile.calculate(dt, new TrapezoidProfile.State(targetPose.getX(), 0), xState);
  //   yState = yProfile.calculate(dt, new TrapezoidProfile.State(targetPose.getY(), 0), yState);

  //   // Create the new desired pose with profiled X & Y motion
  //   Pose2d profiledTarget = new Pose2d(xState.position, yState.position, targetPose.getRotation());

  //   // Compute the desired chassis speeds using the holonomic controller
  //   return profiledHolonomicController.calculate(currentPose, profiledTarget, 0, targetPose.getRotation());
// }

}
