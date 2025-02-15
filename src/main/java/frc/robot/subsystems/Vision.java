package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;

public class Vision extends SubsystemBase {
    private final BiConsumer<String, Object> updateData;
    
    public Vision(BiConsumer<String, Object> updateData) {
        // Example camera initialization - you would add your actual cameras here
        // addCamera("front", "limelight-front", LimelightVersion.V4);
        // addCamera("back", "limelight-back", LimelightVersion.V3);

        this.updateData = updateData;
    }

    @Override
    public void periodic() {
    }
} 