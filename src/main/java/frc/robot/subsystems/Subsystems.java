package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsystems extends SubsystemBase {

    // Instantiate the subsystems (using placeholder motor/control IDs)
    private final Elevator elevator;
    private final Climber climber;
    private final Hopper hopper;
    private final AlgaeIntake algaeIntake;
    private final AlgaeWrist algaeWrist;
    private final EndEffector endEffector;
    
    /**
     * Constructor for the Subsystems container.
     */
    public Subsystems() {
        elevator = new Elevator();
        climber = new Climber();
        hopper = new Hopper();
        algaeIntake = new AlgaeIntake();
        algaeWrist = new AlgaeWrist();
        endEffector = new EndEffector();
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
    
    @Override
    public void periodic() {
        // This container can optionally call periodic() on its subsystems,
        // or you can let the scheduler handle them independently.
    }
} 