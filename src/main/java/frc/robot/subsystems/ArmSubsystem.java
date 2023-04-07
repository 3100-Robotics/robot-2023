package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // Arm viewer
    private final Mechanism2d mech = new Mechanism2d(3, 2); // Create a simulated workspace 2x3 meters
    private final MechanismRoot2d root = mech.getRoot("arm", 1, 0); // Start the arm a little offcenter

    // Arm ligaments (sections)
    private final MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 0, 90));
    private final MechanismLigament2d forearm = elevator.append(new MechanismLigament2d("forearm", 0, -90));
    
    public ArmSubsystem() {
        // Publish the arm simulation to SmartDashboard
        SmartDashboard.putData("Arm", mech);
        elevator.setLength(1.5);
        forearm.setLength(0.5);
    }
}