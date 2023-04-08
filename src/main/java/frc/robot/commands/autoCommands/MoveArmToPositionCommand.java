package frc.robot.commands.autoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPositionCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final Supplier<Translation2d> position; // Using supplier of Translation2d to make tunable positions less of a headache later
    
    /**
     * Create a new command to move the arm to a demanded extention
     * @param robot
     * @param position Translation representing where the arm should be in meters
     */
    public MoveArmToPositionCommand(RobotContainer robot, Supplier<Translation2d> position) {
        arm = robot.armSub;
        this.position = position;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        arm.setTargetPosition(position.get()); // Update the arm with the new demanded positoin
    }

    @Override
    public boolean isFinished() {
        return arm.atTargetPosition();
    }
}
