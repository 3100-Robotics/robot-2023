package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
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
    private final MechanismLigament2d elevatorSim = root.append(new MechanismLigament2d("elevator", 0, 90));
    private final MechanismLigament2d forearmSim = elevatorSim.append(new MechanismLigament2d("forearm", 0, -90));

    private final Elevator elevator = new Elevator();
    private final Arm forearm = new Arm();
    
    public ArmSubsystem() {
        // Publish the arm simulation to SmartDashboard
        SmartDashboard.putData("Arm", mech);

        // Tell the arm to go to a safe position until another is demanded
        setTargetPosition(new Translation2d(0.25, 0.3)); // FIXME: Set to something good
    }

    public void setTargetPosition(Translation2d position) {
        elevator.setTargetHeight(position.getY());
        forearm.setTargetExtension(position.getX());
    }

    public boolean atTargetPosition() {
        return (elevator.atSetpoint() && forearm.atSetpoint()) || RobotBase.isSimulation();
    }

    @Override
    public void periodic() {
        elevatorSim.setLength(elevator.getHeightMeters());
        forearmSim.setLength(forearm.getExtensionMeters());
    }
}