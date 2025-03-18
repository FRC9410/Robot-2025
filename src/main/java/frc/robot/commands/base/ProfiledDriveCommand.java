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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
  private final PIDController TRANS_CONTROLLER;

  private final TrapezoidProfile.Constraints xConstraints;
  private final TrapezoidProfile.Constraints yConstraints;
  
  private TrapezoidProfile.State xState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State yState = new TrapezoidProfile.State(0, 0);

  private final ProfiledPIDController thetaController;
  
  public final Distance AT_POINT_TOLERANCE = Units.Inches.of(0.5);
  public final Angle AT_ROTATION_TOLERANCE = Units.Degrees.of(1);

  public final Distance AUTO_ALIGNMENT_TOLERANCE = Units.Inches.of(1);

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
      table.getEntry("invert paths").setBoolean(false);

      table.getEntry("Trans P").setDouble(3.0);
      table.getEntry("Trans D").setDouble(0.0);
      table.getEntry("Rotation P").setDouble(2.0);
      table.getEntry("Rotation D").setDouble(0.0);

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      //
      //

      thetaController = new ProfiledPIDController(
        2, 0, 0, new TrapezoidProfile.Constraints(CommandSwerveDrivetrain.TURN_SPEED.in(Units.DegreesPerSecond),
            Math.pow(CommandSwerveDrivetrain.TURN_SPEED.in(Units.DegreesPerSecond), 2)));

            
      TRANS_CONTROLLER = new PIDController(
        3,
        0,
        0);
        
        xConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);
        yConstraints = new TrapezoidProfile.Constraints(drivetrain.MAX_SPEED/4, drivetrain.MAX_SPEED);
      
      TRANS_CONTROLLER.setTolerance(AT_POINT_TOLERANCE.in(Units.Meters));

      thetaController.enableContinuousInput(0, 360);
      thetaController.setTolerance(AT_ROTATION_TOLERANCE.in(Units.Degrees));

      profiledHolonomicController = new HolonomicDriveController(
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
    setRotationPid();
      direction = table.getEntry("invert controls").getBoolean(false) ? 1 : -1;
    final boolean useNewFfValues = table.getEntry("useIncreasedFfValue").getBoolean(false);
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
      final ChassisSpeeds chassisSpeeds = profiledHolonomicController.calculate(currentPose, targetPose, 0, targetPose.getRotation());
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
        drivetrain.setControl(drivetrain.FIELD_RELATIVE
          .withVelocityX(chassisSpeeds.vxMetersPerSecond)
          .withVelocityY(chassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
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
    } else if (controller.povLeft().getAsBoolean() || controller.povRight().getAsBoolean() || controller.povUp().getAsBoolean() || controller.povDown().getAsBoolean()) {
      double dpadLeftRightDirection = 0;
      double dpadUpDownDirection = 0;

      if (controller.povLeft().getAsBoolean() || controller.povRight().getAsBoolean()) {
        dpadLeftRightDirection = controller.povRight().getAsBoolean() ? -1.0 : 1.0;
      }
      
      if (controller.povUp().getAsBoolean() || controller.povDown().getAsBoolean()) {
        dpadUpDownDirection = controller.povUp().getAsBoolean() ? 1.0 : -1.0;
      }

      drivetrain.setControl(drivetrain.ROBOT_RELATIVE
        .withVelocityY(dpadLeftRightDirection * drivetrain.MAX_SPEED/10)
        .withVelocityX(dpadUpDownDirection * drivetrain.MAX_SPEED/10)
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
    profiledHolonomicController.getXController().setP(table.getEntry("Trans P").getDouble(3.0));
    profiledHolonomicController.getXController().setD(table.getEntry("Trans D").getDouble(0.0));
    profiledHolonomicController.getYController().setP(table.getEntry("Trans P").getDouble(3.0));
    profiledHolonomicController.getYController().setD(table.getEntry("Trans D").getDouble(0.0));
    profiledHolonomicController.getThetaController().setP(table.getEntry("Rotation P").getDouble(2.0));
    profiledHolonomicController.getThetaController().setD(table.getEntry("Rotation D").getDouble(0.0));
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
