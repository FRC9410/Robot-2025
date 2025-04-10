package frc.robot.commands.actionRequestHandlers;

import java.util.Map;

import frc.robot.subsystems.ActionController;
import frc.robot.subsystems.ActionController.Action;
import frc.team9410.lib.interfaces.ActionRequestHandler;

public class ScoreCoral implements ActionRequestHandler {
    public boolean matches(Map<String, Object> state, Action action) {
        return action.equals(Action.SCORE_CORAL);
    }
    
    public void execute(Map<String, Object> state, ActionController controller) {
        controller.setCommandData(Map.of());
    }
}