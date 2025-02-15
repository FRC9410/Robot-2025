package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
    private final TalonFX endEffectorMotor;
    private static final NeutralOut brake = new NeutralOut();
    private final BiConsumer<String, Object> updateData;

    /**
     * Constructor for the EndEffector subsystem.
     * 
     * 
     */
    public EndEffector(BiConsumer<String, Object> updateData) {
        endEffectorMotor = new TalonFX(Constants.EndEffectorConstants.CAN_ID, Constants.CanBusConstants.CANIVORE_BUS);

        endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);

        this.updateData = updateData;

    }
    
    
    @Override
    public void periodic() {
        
    }
    
    /**
     * Runs the hopper motors at the given speed.
     *
     * The primary motor is commanded to run at the specified speed while the secondary
     * motor runs in reverse (i.e. with the negative of the speed).
     *
     * @param speed The desired motor output (range between -1 and 1).
     */
    public void runEndEffector(double speed) {
        endEffectorMotor.setVoltage(speed);
        // No need to command secondaryMotor here since it follows primaryMotor in reverse.
    }
    
    /**
     * Stops both hopper motors.
     */
    public void stopEndEffector() {
        endEffectorMotor.setControl(brake);
    }
} 