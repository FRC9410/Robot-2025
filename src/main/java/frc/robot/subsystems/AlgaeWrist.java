package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeWristConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class AlgaeWrist extends SubsystemBase {
    private final TalonFX wristMotor;
    private final CANcoder cancoder;
    private final BiConsumer<String, Object> updateData;
    private double voltage;
    private double setpoint;
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    /**
     * Constructor for the AlgaeWrist subsystem.
     *
     * @param motorID   CAN ID for the wrist motor.
     * @param encoderID CAN ID for the absolute encoder.
     */
    public AlgaeWrist(BiConsumer<String, Object> updateData) {
        wristMotor = new TalonFX(Constants.AlgaeWristConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
        cancoder = new CANcoder(Constants.AlgaeWristConstants.ENCODER_CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(-0.1));

        cancoder.getConfigurator().apply(encoderConfig);
        
        // Configure PID
        config.Slot0.kP = Constants.AlgaeWristConstants.kP;
        config.Slot0.kI = Constants.AlgaeWristConstants.kI;
        config.Slot0.kD = Constants.AlgaeWristConstants.kD;
        config.Slot0.kG = Constants.AlgaeWristConstants.kG;
        config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = 2.0;
        config.Feedback.RotorToSensorRatio = 50.0;


        wristMotor.getConfigurator().apply(config);

        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.AlgaeWristConstants.MOTION_MAGIC_VELOCITY)
          .withMotionMagicAcceleration(Constants.AlgaeWristConstants.MOTION_MAGIC_ACCELERATION);

        wristMotor.getConfigurator().apply(motionMagicConfigs);

        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(100, cancoder.getPosition(), cancoder.getVelocity());
        wristMotor.setNeutralMode(NeutralModeValue.Brake);
          
        this.updateData = updateData;
        setpoint = 0.8;
        wristMotor.setControl(motionMagicRequest.withPosition(0.07).withSlot(0));
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
    public void setPosition(double position) {
        wristMotor.setControl(motionMagicRequest.withPosition(position).withSlot(0));
    }
} 