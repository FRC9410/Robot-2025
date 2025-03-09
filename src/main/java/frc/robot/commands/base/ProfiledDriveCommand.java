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
import frc.robot.subsystems.Elevator;
import frc.robot.utils.Utils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProfiledDriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private HolonomicDriveController holonomicController;
  private double direction;
  private Dashboard dashboard;
  private final Elevator elevator;
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final ActionController actionController;



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

  ///////////////////////////////////////////////////////////////////////////////////////
  //
  //  Profiled Motion Testing END
  //
  ///////////////////////////////////////////////////////////////////////////////////////

  /** Creates a new DriveCommand. */
  public ProfiledDriveCommand(CommandSwerveDrivetrain drivetrain,
    CommandXboxController controller,
    Dashboard dashboard,
    ActionController actionController,
    Elevator elevator) {
      inst = NetworkTableInstance.getDefault();
      table = inst.getTable("Driving PIDs");

      this.drivetrain = drivetrain;
      this.controller = controller;
      this.dashboard = dashboard;
      this.actionController = actionController;
      this.elevator = elevator;
      this.holonomicController = new HolonomicDriveController(
        new PIDController(0.6, 0, 0.01),
        new PIDController(0.6, 0, 0.01),
        new ProfiledPIDController(1, 0, 0.00025,
              new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE*2, drivetrain.MAX_ANGULAR_RATE*4)));
      // direction = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1.0 : 1.0;//
      table.getEntry("invert controls").setBoolean(false);

      // table.getEntry("X P").setDouble(0.5);
      // table.getEntry("X D").setDouble(0.01);
      // table.getEntry("Y P").setDouble(0.5);
      // table.getEntry("Y D").setDouble(0.01);
      // table.getEntry("Rotation P").setDouble(01);
      // table.getEntry("Rotation D").setDouble(0.00025);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      //
      //

      thetaController = new ProfiledPIDController(1, 0, 0.00025,
        new TrapezoidProfile.Constraints(drivetrain.MAX_ANGULAR_RATE*2, drivetrain.MAX_ANGULAR_RATE*4));
        
        xConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);
        yConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);

      profiledHolonomicController = new HolonomicDriveController(
        new PIDController(0.8, 0, 0.01),  // X Translation PID
        new PIDController(0.8, 0, 0.01),  // Y Translation PID
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
    // setRotationPid();
      direction = table.getEntry("invert controls").getBoolean(false) ? 1 : -1;
    final Pose2d currentPose = drivetrain.getState().Pose;
    if (currentPose != null
    && actionController.getCommandField(Constants.MapConstants.TARGET_POSE) != null
    && Math.abs(controller.getLeftX()) < 0.10
    && Math.abs(controller.getLeftY()) < 0.10
    && Math.abs(controller.getRightX()) < 0.10) {
      // final Pose2d requestedPose = (Pose2d) actionController.getCommandField(Constants.MapConstants.TARGET_POSE);
      // final Pose2d targetPose = Utils.getNextPose(currentPose, requestedPose);
      final Pose2d targetPose = (Pose2d) actionController.getCommandField(Constants.MapConstants.TARGET_POSE);
      // final ChassisSpeeds ChassisSpeeds = holonomicController.calculate(
      //   currentPose,
      //   targetPose,
      //   0.0,
      //   targetPose.getRotation()
      // );
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

        final double maxSpeed = 0.9 * drivetrain.MAX_SPEED;
        final double minSpeed = 0.0 * drivetrain.MAX_SPEED;

        final double maxTurnSpeed = 0.9 * drivetrain.MAX_ANGULAR_RATE;
        final double minTurnSpeed = 0.0 * drivetrain.MAX_ANGULAR_RATE;

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
        // if (Math.abs(controller.getLeftX()) > 0.10
        // || Math.abs(controller.getLeftY()) > 0.10
        // || Math.abs(controller.getRightX()) > 0.10) {
        //   actionController.setAutoMode(false);
        // }
    } else if (elevator.getCurrentHeight() > 5) {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
        // if (Math.abs(controller.getLeftX()) > 0.10
        // || Math.abs(controller.getLeftY()) > 0.10
        // || Math.abs(controller.getRightX()) > 0.10) {
        //   actionController.setAutoMode(false);
        // }
    } else {
      drivetrain.setControl(drivetrain.FIELD_RELATIVE
        .withVelocityX(direction * Utils.cubeInput(controller.getLeftY()) * drivetrain.MAX_SPEED)
        .withVelocityY(direction * Utils.cubeInput(controller.getLeftX()) * drivetrain.MAX_SPEED)
        .withRotationalRate(getRotation()));
        // if (Math.abs(controller.getLeftX()) > 0.10
        // || Math.abs(controller.getLeftY()) > 0.10
        // || Math.abs(controller.getRightX()) > 0.10) {
        //   actionController.setAutoMode(false);
        // }
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
    profiledHolonomicController.getXController().setP(table.getEntry("X P").getDouble(0.006));
    profiledHolonomicController.getXController().setD(table.getEntry("X D").getDouble(0));
    profiledHolonomicController.getYController().setP(table.getEntry("Y P").getDouble(0.006));
    profiledHolonomicController.getYController().setD(table.getEntry("Y D").getDouble(0));
    profiledHolonomicController.getThetaController().setP(table.getEntry("Rotation P").getDouble(0.006));
    profiledHolonomicController.getThetaController().setD(table.getEntry("Rotation D").getDouble(0));
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
