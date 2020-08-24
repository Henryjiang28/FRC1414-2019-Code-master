package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Arm.ArmClimbCmd;
import frc.robot.commands.Arm.ArmCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class EndClimbCmdGroup extends CommandGroup {
    public EndClimbCmdGroup() {
        // addParallel(new ClimbPositionCmd(), 10);
        addSequential(new ArmCmd(ArmPosition.Climb));
        addSequential(new ElevatorCmd(ElevatorPosition.Starting));
        addSequential(new ArmClimbCmd());
    }
}