package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MapConstants;
import frc.robot.commands.actionRequestHandlers.*;
import frc.robot.subsystems.ActionController.Action;
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
    private boolean autoMode;
    private Idle idleHandler = new Idle();
  
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
        idleHandler
    );
    
  
    private final List<ActionRequestHandler> autoRequestHandlers = List.of(
        new AutoElevator(),
        new AutoIntaking(),
        new AutoDriving(),
        idleHandler
    );

    public ActionController(Map<String, Object> subsytemData, CommandSwerveDrivetrain drivetrain) {
        // Initialization code for the action controller subsystem
        commandData = new HashMap<>();
        this.subsystemData = subsytemData;
        this.drivetrain = drivetrain;
        this.autoMode = false;
    }   

    @Override
    public void periodic() {
        subsystemData.put(MapConstants.POSE, drivetrain.getState().Pose);

        System.out.println(autoMode);

        if (autoMode) {
            doAutoRequest(subsystemData);
        } else {
            idleHandler.execute(subsystemData, this);
        }
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
            if (handler.matches(state, Action.IDLE)) {
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
        } else if (value instanceof Pose2d) {
            return (Pose2d) value;
        } else {
            return null; // Handle cases where the value is not of expected type
        }
    }
    
    public void toggleAutoMode() {
        if (!autoMode) {
            setCommandData(Map.of());
        }

        autoMode = !autoMode;
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