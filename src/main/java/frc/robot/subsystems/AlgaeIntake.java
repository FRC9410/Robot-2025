package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private static final NeutralOut brake = new NeutralOut();
    private final BiConsumer<String, Object> updateData;
    private double voltage;
    
    /**
     * Constructor for the Algae Intake subsystem.
     * 
     * @param motorID CAN ID for the intake motor.
     */
    public AlgaeIntake(BiConsumer<String, Object> updateData) {
        intakeMotor = new TalonFX(Constants.AlgaeIntakeConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

        // Configure motor controller using the configTalonFx method.
        configTalonFx();

        // Set neutral mode; coast mode is often preferred for an intake.
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        this.updateData = updateData;

        voltage = Constants.AlgaeIntakeConstants.STOP_VOLTAGE;
    }
    
    /**
     * Runs the intake motor at the specified speed.
     * <p>
     * Positive speeds might be used for intake, while negative speeds reverse the motor
     * for outtake. Adjust your usage per your mechanism's design.
     * </p>
     * 
     * @param speed The motor output in the range [-1, 1].
     */
    public void setVoltage(double speed) {
        intakeMotor.setControl(new DutyCycleOut(-speed));
        // if (voltage != this.voltage) {
        //     this.voltage = voltage;
        //     intakeMotor.setVoltage(voltage);
        // }
    }
    
    @Override
    public void periodic() {
    }

    /**
     * Holds (stalls) the intake by commanding the motor to maintain its current position
     * using closed-loop PID control.
     */
    public void stallIntake() {
    }

    /**
     * Configures the TalonFX motor controller with default settings.
     * Adjust any motor-specific tuning parameters as needed.
     */
    private void configTalonFx() {
         TalonFXConfiguration config = new TalonFXConfiguration();
         // Set slot 0 PID constants using values from Constants
         config.Slot0.kP = Constants.AlgaeIntakeConstants.kP;
         config.Slot0.kI = Constants.AlgaeIntakeConstants.kI;
         config.Slot0.kD = Constants.AlgaeIntakeConstants.kD;
         config.Slot0.kG = Constants.AlgaeIntakeConstants.kF;
         // Apply the configuration
         intakeMotor.getConfigurator().apply(config);
    }
} 