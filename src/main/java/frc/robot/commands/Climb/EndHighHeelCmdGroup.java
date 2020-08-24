package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.Arm.ArmCmd;
import frc.robot.commands.Arm.ArmTeleopCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class EndHighHeelCmdGroup extends CommandGroup { // CHANGE
    public EndHighHeelCmdGroup() {
        addSequential(new ClimbHookCmd(false), 1);
        addParallel(new ArmCmd(ArmPosition.Hatch));
        addSequential(new ElevatorCmd(ElevatorPosition.Hatch2));
        addSequential(new ClimbTeleopCmd());
    }
}