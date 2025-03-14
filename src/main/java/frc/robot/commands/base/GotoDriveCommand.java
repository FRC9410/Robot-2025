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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GotoDriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final NetworkTableInstance inst;
  private final NetworkTable table;



  ///////////////////////////////////////////////////////////////////////////////////////
  //
  //  Profiled Motion Testing
  //
  ///////////////////////////////////////////////////////////////////////////////////////

  
  private final HolonomicDriveController profiledHolonomicController;

  private final TrapezoidProfile.Constraints xConstraints;
  private final TrapezoidProfile.Constraints yConstraints;
  
  private TrapezoidProfile.State xState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State yState = new TrapezoidProfile.State(0, 0);

  private final ProfiledPIDController thetaController;
  private final Pose2d targetPose;

  ///////////////////////////////////////////////////////////////////////////////////////
  //
  //  Profiled Motion Testing END
  //
  ///////////////////////////////////////////////////////////////////////////////////////

  /** Creates a new DriveCommand. */
  public GotoDriveCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
      this.targetPose = targetPose;
      this.drivetrain = drivetrain;
      inst = NetworkTableInstance.getDefault();
      table = inst.getTable("Driving Command");

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      //
      //

      thetaController = new ProfiledPIDController(1, 0, 0.00025,
        new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE*2, drivetrain.MAX_ANGULAR_RATE*4));
        
        xConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);
        yConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);

      profiledHolonomicController = new HolonomicDriveController(
        new PIDController(0.5, 0, 0.01),  // X Translation PID
        new PIDController(0.5, 0, 0.01),  // Y Translation PID
        thetaController);                // Rotation ProfiledPID

      thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
      final ChassisSpeeds ChassisSpeeds = calculate(currentPose, targetPose, 0.02);
      final double xDelta = targetPose.getTranslation().getX() - currentPose.getTranslation().getX();
      final double yDelta = targetPose.getTranslation().getY() - currentPose.getTranslation().getY();
      final double rotationDelta = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
      
      table.getEntry("Y Delta").setDouble(xDelta);
      table.getEntry("X Delta").setDouble(yDelta);
      table.getEntry("Rotation Delta").setDouble(rotationDelta);

      final boolean isWithinTolerance = Math.abs(xDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE &&
        Math.abs(yDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE
        && Math.abs(rotationDelta) < Constants.AutoConstants.ROTATION_TOLERANCE;

      if (!isWithinTolerance) {
        final double xChassisSpeed = ChassisSpeeds.vxMetersPerSecond * drivetrain.MAX_SPEED;
        final double yChassisSpeed = ChassisSpeeds.vyMetersPerSecond * drivetrain.MAX_SPEED;
        final double chassisTurnSpeed = ChassisSpeeds.omegaRadiansPerSecond * drivetrain.MAX_ANGULAR_RATE;

        final double xSpeedDirection = xChassisSpeed > 1.0 ? 1 : -1;
        final double ySpeedDirection = yChassisSpeed > 1.0 ? 1 : -1;
        final double turnSpeedDirection = chassisTurnSpeed > 1.0 ? 1 : -1;

        final double maxSpeed = 0.8 * drivetrain.MAX_SPEED;
        final double minSpeed = 0.07 * drivetrain.MAX_SPEED;

        final double maxTurnSpeed = 0.8 * drivetrain.MAX_ANGULAR_RATE;
        final double minTurnSpeed = 0.06 * drivetrain.MAX_ANGULAR_RATE;

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
          .withVelocityX(xSpeed + (0.04 * xSpeedDirection))
          .withVelocityY(ySpeed + (0.04 * ySpeedDirection))
          .withRotationalRate(turnSpeed + (0.02 * turnSpeedDirection)));
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

    final boolean isWithinTolerance = Math.abs(xDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE + 0.1 &&
      Math.abs(yDelta) < Constants.AutoConstants.TRANSLATION_TOLERANCE + 0.1
      && Math.abs(rotationDelta) < Constants.AutoConstants.ROTATION_TOLERANCE + 5;

    if (isWithinTolerance) {
      return true;
    }
    return false;
  }

  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose, double dt) {
    // Generate trapezoidal profiles for X and Y
    TrapezoidProfile xProfile = new TrapezoidProfile(xConstraints);
    TrapezoidProfile yProfile = new TrapezoidProfile(yConstraints);

    // Update profile states
    xState = xProfile.calculate(dt, new TrapezoidProfile.State(targetPose.getX(), 0), xState);
    yState = yProfile.calculate(dt, new TrapezoidProfile.State(targetPose.getY(), 0), yState);

    // Create the new desired pose with profiled X & Y motion
    Pose2d profiledTarget = new Pose2d(xState.position, yState.position, targetPose.getRotation());

    // Compute the desired chassis speeds using the holonomic controller
    return profiledHolonomicController.calculate(currentPose, profiledTarget, 0, targetPose.getRotation());
}

}
