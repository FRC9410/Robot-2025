package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorFollowerMotor;
    private final PositionVoltage positionRequest;
    private final BiConsumer<String, Object> updateData;
    private double voltage;
    private double setpoint;

    public Elevator(BiConsumer<String, Object> updateData) {
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
        elevatorFollowerMotor = new TalonFX(Constants.ElevatorConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);
        
        // Create the position request for closed-loop control
        positionRequest = new PositionVoltage(0).withSlot(0);

        // Configure the Kraken
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        
        // Configure PID
        config.Slot0.kP = Constants.ElevatorConstants.kP;
        config.Slot0.kI = Constants.ElevatorConstants.kI;
        config.Slot0.kD = Constants.ElevatorConstants.kD;
        config.Slot0.kG = Constants.ElevatorConstants.kG;

        // Configure soft limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorConstants.L4_SCORE_POSITION;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorConstants.HOME_POSITION;

        // Apply configuration
        elevatorMotor.getConfigurator().apply(config);
        elevatorFollowerMotor.getConfigurator().apply(followerConfig);

        // Set brake mode
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

        elevatorFollowerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), true));

        this.updateData = updateData;

        voltage = Constants.ElevatorConstants.STOP_VOLTAGE;
        setpoint = Constants.ElevatorConstants.HOME_POSITION;
    }

    @Override
    public void periodic() {
    }

    /**
     * Sets the elevator to a specific height
     * @param heightMeters The target height in meters
     */
    public void setPosition(double position) {
        if(position != setpoint) {
            setpoint = position;
            elevatorMotor.setControl(positionRequest.withPosition(position));
        }
    }

    /**
     * Gets the current height of the elevator
     * @return Current height in meters
     */
    public double getCurrentHeight() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns whether the elevator is at its target position
     */
    public boolean atTargetPosition() {
        return Math.abs(elevatorMotor.getClosedLoopError().getValue()) < 0.02; // 2cm tolerance
    }

    public void setVoltage(double voltage) {
        if (voltage != this.voltage) {
            this.voltage = voltage;
            elevatorMotor.setVoltage(voltage);
        }
    }

    /**
     * Zeros the elevator encoder at the current position
     * Use this when the elevator is at a known position
     */
    public void zeroEncoder() {
        elevatorMotor.setPosition(0);
    }
} 