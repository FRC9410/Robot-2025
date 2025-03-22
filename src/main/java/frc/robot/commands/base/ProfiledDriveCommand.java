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

  
  private final HolonomicDriveController holonomicController;
  private final PIDController TRANS_CONTROLLER;

  private final ProfiledPIDController thetaController;
  private double transFF = 0;
  private double rotationFF = 0;
  
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
      // direction = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1.0 : 1.0;//
      table.getEntry("invert controls").setBoolean(false);
      table.getEntry("invert paths").setBoolean(false);

      table.getEntry("Trans P").setDouble(0.5);
      table.getEntry("Trans D").setDouble(0.0);
      table.getEntry("Rotation P").setDouble(0.7);
      table.getEntry("Rotation D").setDouble(0.0);
      table.getEntry("TransFF").setDouble(0.25);
      table.getEntry("RotationFF").setDouble(0.3);

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
    setRotationPid();
    direction = table.getEntry("invert controls").getBoolean(false) ? 1 : -1;
    final Pose2d currentPose = drivetrain.getState().Pose;

    if (currentPose != null
    && actionController.getCommandField(Constants.MapConstants.TARGET_POSE) != null
    && Math.abs(controller.getLeftX()) < 0.10
    && Math.abs(controller.getLeftY()) < 0.10
    && Math.abs(controller.getRightX()) < 0.10) {
      final Pose2d targetPose = (Pose2d) actionController.getCommandField(Constants.MapConstants.TARGET_POSE);
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
          .withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(turnSpeed));
      }
    } else if (dashboard.getIsClimbing()) {
      drivetrain.setControl(drivetrain.FF_FIELD_RELATIVE
        .withVelocityX(direction * Utils.squareInput(controller.getLeftY()) * drivetrain.MAX_SPEED/5)
        .withVelocityY(direction * Utils.squareInput(controller.getLeftX()) * drivetrain.MAX_SPEED/5)
        .withRotationalRate(getRotation()));
    } else if (controller.povLeft().getAsBoolean() || controller.povRight().getAsBoolean() || controller.povUp().getAsBoolean() || controller.povDown().getAsBoolean()) {
      double dpadLeftRightDirection = 0;
      double dpadUpDownDirection = 0;
      double rotationDirection = 0;

      if (controller.povLeft().getAsBoolean() || controller.povRight().getAsBoolean()) {
        dpadLeftRightDirection = controller.povRight().getAsBoolean() ? -1.0 : 1.0;
      }
      
      if (controller.povUp().getAsBoolean() || controller.povDown().getAsBoolean()) {
        dpadUpDownDirection = controller.povUp().getAsBoolean() ? 1.0 : -1.0;
      }
      
      rotationDirection = getRotation() > 0 ? 1.0 : -1.0;

      drivetrain.setControl(drivetrain.ROBOT_RELATIVE
        .withVelocityY(dpadLeftRightDirection * drivetrain.MAX_SPEED/10)
        .withVelocityX(dpadUpDownDirection * drivetrain.MAX_SPEED/10)
        .withRotationalRate(getRotation()));
    } else if (elevator.getCurrentHeight() > 5) {
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
    holonomicController.getXController().setP(table.getEntry("Trans P").getDouble(0.0));
    holonomicController.getXController().setD(table.getEntry("Trans D").getDouble(0.0));
    holonomicController.getYController().setP(table.getEntry("Trans P").getDouble(0.0));
    holonomicController.getYController().setD(table.getEntry("Trans D").getDouble(0.0));
    holonomicController.getThetaController().setP(table.getEntry("Rotation P").getDouble(0.0));
    holonomicController.getThetaController().setD(table.getEntry("Rotation D").getDouble(0.0));
    transFF = table.getEntry("TransFF").getDouble(0.0);
    rotationFF = table.getEntry("RotationFF").getDouble(0.0);
  }
}
