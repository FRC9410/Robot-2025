package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MapConstants;
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
    private Map<String, Object> subsystemData;
    private CommandSwerveDrivetrain drivetrain;
  
    private final List<ActionRequestHandler> requestHandlers = List.of(
        new Climb(),
        new DeployClimber(),
        new IntakeAlgaeGround(),
        new IntakeAlgaeReef(),
        new IntakeCoral(),
        new LayupAlgae(),
        new PlaceAlgae(),
        new ProcessAlgae(),
        new ScoreCoral(),
        new Idle()
    );
    
  
    private final List<ActionRequestHandler> autoRequestHandlers = List.of(
        new AutoIntaking(),
        new Idle()
    );

    public ActionController(Map<String, Object> subsytemData, CommandSwerveDrivetrain drivetrain) {
        // Initialization code for the action controller subsystem
        commandData = new HashMap<>();
        this.subsystemData = subsytemData;
        this.drivetrain = drivetrain;

    }   

    @Override
    public void periodic() {
        subsystemData.put(MapConstants.POSE, drivetrain.getState().Pose);

        doAutoRequest(subsystemData);
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
    
    /**
     * Example method that performs an action.
     */
    public void doAutoRequest(Map<String, Object> state) {
        for (ActionRequestHandler handler : autoRequestHandlers) {
            if (handler.matches(state, null)) {
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
        CLIMB,
        DEPLOY_CLIMBER,
        INTAKE_ALGAE_GROUND,
        INTAKE_ALGAE_REEF,
        INTAKE_CORAL,
        LAYUP_ALGAE,
        PLACE_ALGAE,
        PROCESS_ALGAE,
        SCORE_CORAL,
        DEV_MODE,
        DEMO_MODE
    }
} 