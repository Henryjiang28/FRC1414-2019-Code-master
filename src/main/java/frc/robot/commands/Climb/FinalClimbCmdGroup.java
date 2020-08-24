package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Arm.ArmClimbCmd;
import frc.robot.commands.Elevator.ElevatorClimbCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class FinalClimbCmdGroup extends CommandGroup {
    public FinalClimbCmdGroup() {
        // addParallel(new ClimbDriveCmd(0.2), 10);
        addSequential(new ClimbHookCmd(false), 1);
        addSequential(new ElevatorCmd(ElevatorPosition.Final));
        addSequential(new ElevatorClimbCmd());
        addSequential(new ArmClimbCmd());
    }
}