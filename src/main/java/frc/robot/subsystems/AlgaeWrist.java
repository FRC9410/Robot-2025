package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeWristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

public class AlgaeWrist extends SubsystemBase {
    private final SparkClosedLoopController pidController;
    private final SparkMax wristMotor;
    private final SparkMaxConfig config;
    /**
     * Constructor for the AlgaeWrist subsystem.
     *
     * @param motorID   CAN ID for the wrist motor.
     * @param encoderID CAN ID for the absolute encoder.
     */
    public AlgaeWrist() {
        wristMotor = new SparkMax(AlgaeWristConstants.CAN_ID, MotorType.kBrushless);
        pidController = wristMotor.getClosedLoopController();

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(
                AlgaeWristConstants.kP,
                AlgaeWristConstants.kI,
                AlgaeWristConstants.kD
            )
            .outputRange(AlgaeWristConstants.MIN_ANGLE_UNITS,
                AlgaeWristConstants.MAX_ANGLE_UNITS);
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        config.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(1000)
            .maxAcceleration(1000)
            .allowedClosedLoopError(1)
            // Set MAXMotion parameters for velocity control in slot 1
            .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);
            
        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setReference(AlgaeWristConstants.MIN_ANGLE_UNITS, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0);
    }
    
    @Override
    public void periodic() {
       
    }

    /**
     * Commands the wrist to move to a specific target angle (in degrees).
     * The target angle is clamped between WRIST_MIN_ANGLE_DEGREES and WRIST_MAX_ANGLE_DEGREES.
     * The method converts the desired angle into motor rotations (using the gear ratio) and
     * then sets the internal motor controller's position setpoint.
     *
     * @param targetAngleDegrees The desired target angle in degrees.
     */
    public void setAngle(double targetUnits) {
        pidController.setReference(targetUnits, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
} 