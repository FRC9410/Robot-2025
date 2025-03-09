package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import frc.robot.Constants;
import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;
import frc.team9410.lib.interfaces.ActionRequestHandler;

public class Idle implements ActionRequestHandler {
    public boolean matches(Map<String, Object> state, Action action) {
        return action.equals(Action.IDLE);
    }
    
    public void execute(Map<String, Object> state, ActionController controller) {
        final Object position = controller.getCommandField(Constants.MapConstants.ELEVATOR_POSITION);
        if (position != null) {
            controller.setCommandData(Map.of(Constants.MapConstants.ELEVATOR_POSITION, position));
        } else {
            controller.setCommandData(Map.of());
        }
    }
}