package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

public class Subsystems {

    // Instantiate the subsystems (using placeholder motor/control IDs)
    private final Elevator elevator;
    private final Climber climber;
    private final Hopper hopper;
    private final AlgaeIntake algaeIntake;
    private final AlgaeWrist algaeWrist;
    private final EndEffector endEffector;
    private final ActionController actionController;
    private final Map<String, Object> subsystemData;
    private final Vision vision;
    private final Sensors sensors;
    
    /**
     * Constructor for the Subsystems container.
     */
    public Subsystems() {
        elevator = new Elevator((key, value) -> updateSubsystemData(key, value));
        climber = new Climber((key, value) -> updateSubsystemData(key, value));
        hopper = new Hopper((key, value) -> updateSubsystemData(key, value));
        algaeIntake = new AlgaeIntake((key, value) -> updateSubsystemData(key, value));
        algaeWrist = new AlgaeWrist((key, value) -> updateSubsystemData(key, value));
        endEffector = new EndEffector((key, value) -> updateSubsystemData(key, value));
        vision = new Vision((key, value) -> updateSubsystemData(key, value));
        sensors = new Sensors((key, value) -> updateSubsystemData(key, value));
        
        subsystemData = new HashMap<>();
        actionController = new ActionController();
    }
    
    // Accessor methods to retrieve subsystems:
    
    public Elevator getElevator() {
        return elevator;
    }
    
    public Climber getClimber() {
        return climber;
    }
    
    public Hopper getHopper() {
        return hopper;
    }
    
    public AlgaeIntake getAlgaeIntake() {
        return algaeIntake;
    }
    
    public AlgaeWrist getAlgaeWrist() {
        return algaeWrist;
    }
    
    public EndEffector getEndEffector() {
        return endEffector;
    }
    
    public ActionController getActionController() {
        return actionController;
    }

    public Vision getVision() {
        return vision;
    }

    public void updateSubsystemData(String key, Object value) {
        subsystemData.put(key, value);
    }

    public Object getSubsystemField(String key) {
        return subsystemData.get(key);
    }

    public Map<String, Object> getSubsystemData() {
        return subsystemData;
    }
} 