package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.actionRequestHandlers.*;
import frc.team9410.lib.interfaces.ActionRequestHandler;

/**
 * ActionController subsystem.
 * 
 * This subsystem is designed to handle high-level actions or behaviors
 * that are not tied directly to a single motor or sensor. Adjust its methods
 * as needed for your robot.
 */
public class ActionController extends SubsystemBase {
    private Map<String, Object> commandData;
    
  
    private final List<ActionRequestHandler> requestHandlers = List.of(
        new Idle()
    );

    public ActionController() {
        // Initialization code for the action controller subsystem
        commandData = new HashMap<>();
    }   

    @Override
    public void periodic() {
        // Put code here to be run every scheduling cycle
    }
    
    /**
     * Example method that performs an action.
     */
    public void doRequest(Map<String, Object> state, Action action) {
        for (ActionRequestHandler handler : requestHandlers) {
            if (handler.matches(state, action)) {
                handler.execute(state, this);
                break;
            }
        }
    }
    public void setCommandData(Map<String, Object> newCommandData) {
        this.commandData = newCommandData;
    }

    public void updateCommandData(String key, Object value) {
      commandData.put(key, value);
    }
  
    public void removeMultipleCommandKeys(List<String> keys) {
      for (String key : keys) {
        commandData.remove(key);
      }
    }

    public Map<String, Object> getCommandData() {
        return commandData;
    }

    public Object getCommandField(String key) {
        Object value = commandData.get(key);
  
        if (value instanceof String) {
            return (String) value;
        } else if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            return null; // Handle cases where the value is not of expected type
        }
    }
  

    public enum Action {
      IDLE,
      DEV_MODE,
      DEMO_MODE
    }
} 