package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Arm.Arm2ClimbCmd;
import frc.robot.commands.Arm.ArmClimbCmd;
import frc.robot.commands.Elevator.Elevator2ClimbCmd;
import frc.robot.commands.Elevator.ElevatorClimbCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class Final2ClimbCmdGroup extends CommandGroup {
    public Final2ClimbCmdGroup() {
        // addParallel(new ClimbDriveCmd(0.2), 10);
        addSequential(new ClimbHookCmd(false), 1);
        addSequential(new ElevatorCmd(ElevatorPosition.HookOn));
        addSequential(new Elevator2ClimbCmd());
        addSequential(new Arm2ClimbCmd());
    }
}