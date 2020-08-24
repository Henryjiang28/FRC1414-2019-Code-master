package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.Arm.ArmCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class HighHeelCmdGroup extends CommandGroup { // CHANGE
    public HighHeelCmdGroup() {
        addSequential(new ClimbHookCmd(false), 0.5);
        addParallel(new ArmCmd(ArmPosition.Hatch));
        addSequential(new ElevatorCmd(ElevatorPosition.Hatch2));
        addSequential(new WaitCommand(0.5));
        addSequential(new ClimbHookCmd(true), 0.5);
        addSequential(new ElevatorCmd(ElevatorPosition.HighHeels));
        addSequential(new ClimbTeleopCmd());
    }
}