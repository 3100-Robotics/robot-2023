package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase{

    // var setup
    Arm arm;
    boolean down;

    public MoveArm(Arm arm, boolean isBack) {
        // typical stuff
        this.arm = arm;
        this.down = isBack;
    }

    @Override
    public void initialize() {
        // move the setpoint
        arm.incrementSetpoint(down);
    }

    @Override
    public boolean isFinished() {
        // done after one run
        return true;
    }
}
