package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class AlgaeWrist extends SubsystemBase {
    private final SparkClosedLoopController pidController;
    private AbsoluteEncoder encoder;
    private final SparkMax wristMotor;
    private final SparkMaxConfig config;
    private final BiConsumer<String, Object> updateData;
    private double voltage;
    private double setpoint;
    /**
     * Constructor for the AlgaeWrist subsystem.
     *
     * @param motorID   CAN ID for the wrist motor.
     * @param encoderID CAN ID for the absolute encoder.
     */
    public AlgaeWrist(BiConsumer<String, Object> updateData) {
        wristMotor = new SparkMax(AlgaeWristConstants.CAN_ID, MotorType.kBrushless);
        pidController = wristMotor.getClosedLoopController();
        encoder = wristMotor.getAbsoluteEncoder();

        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(20).inverted(true);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(
                AlgaeWristConstants.kP,
                AlgaeWristConstants.kI,
                AlgaeWristConstants.kD
            )
            .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);
        config.absoluteEncoder
            .zeroOffset(0)
          .positionConversionFactor(1)
          .velocityConversionFactor(1);
            
        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidController.setReference(0.8, ControlType.kPosition);
          
        this.updateData = updateData;

        voltage = AlgaeWristConstants.STOP_VOLTAGE;
        setpoint = 0.8;
    }
    
    @Override
    public void periodic() {
        updateData.accept("wristVoltage", wristMotor.getOutputCurrent());
        updateData.accept("wristPosition", encoder.getPosition());
        updateData.accept("wristSetpoint", wristMotor.getAppliedOutput());
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
        if (position != setpoint) {
            setpoint = position;
            pidController.setReference(position, ControlType.kPosition);
        }
    }

    public void setVoltage(double voltage) {
        if (voltage != this.voltage) {
            this.voltage = voltage;
            wristMotor.setVoltage(voltage);
        }
    }
} 