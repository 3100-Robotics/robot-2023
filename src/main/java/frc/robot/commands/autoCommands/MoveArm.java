package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase{
    Arm arm;
    boolean down;

    public MoveArm(Arm arm, boolean isBack) {
        this.arm = arm;
        this.down = isBack;
    }

    @Override
    public void initialize() {
        arm.incrementSetpoint(down);
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }
}
