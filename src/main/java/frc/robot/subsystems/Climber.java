package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final TalonFX climberMotor;
    private final PositionVoltage positionRequest;

    /**
     * Constructor for the Climber subsystem.
     *
     * @param motorID CAN ID for the climber's motor.
     */
    public Climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
        positionRequest = new PositionVoltage(0).withSlot(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.ClimberConstants.kP;
        config.Slot0.kI = Constants.ClimberConstants.kI;
        config.Slot0.kD = Constants.ClimberConstants.kD;
        config.Slot0.kG = Constants.ClimberConstants.kF;

        // Set up soft limits to prevent over-extension or over-retraction.
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.WINCH_CLIMB_UNITS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        climberMotor.getConfigurator().apply(config);

        // Set brake mode so the winch holds its position when no power is applied.
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
    }

    /**
     * Commands the climber to extend or retract to a specified position (in meters).
     *
     * @param extensionMeters The desired extension in meters.
     */
    public void setPosition(double extensionUnits) {
        climberMotor.setControl(positionRequest.withPosition(extensionUnits));
    }

    /**
     * Returns the current climber extension (in meters) based on the motor encoder.
     *
     * @return Current extension in meters.
     */
    public double getCurrentExtension() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    /**
     * Zeroes the climber encoder. Call this when the climber is at a known reference position.
     */
    public void zeroEncoder() {
        climberMotor.setPosition(0);
    }
} 