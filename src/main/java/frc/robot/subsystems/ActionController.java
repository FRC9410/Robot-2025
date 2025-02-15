package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ActionController subsystem.
 * 
 * This subsystem is designed to handle high-level actions or behaviors
 * that are not tied directly to a single motor or sensor. Adjust its methods
 * as needed for your robot.
 */
public class ActionController extends SubsystemBase {
    private final BiConsumer<String, Object> updateData;
    Function<String, Object> getSubsystemData;

    public ActionController(BiConsumer<String, Object> updateData, Function<String, Object> getSubsystemData) {
        // Initialization code for the action controller subsystem
        this.updateData = updateData;
        this.getSubsystemData = getSubsystemData;
    }   
    @Override
    public void periodic() {
        // Put code here to be run every scheduling cycle
    }
    
    /**
     * Example method that performs an action.
     */
    public void performAction() {
        // Insert action logic here.
    }
} 