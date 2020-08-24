
package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.Arm.ArmCmd;
import frc.robot.commands.Elevator.ElevatorCmd;
import frc.robot.commands.Intake.Hatch2Cmd;
import frc.robot.commands.Intake.HatchCmd;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class StartClimbCmdGroup extends CommandGroup {
    public StartClimbCmdGroup() {
        addSequential(new ClimbHookCmd(false), 0.5);
        addParallel(new ArmCmd(ArmPosition.HookOn));
        addSequential(new ElevatorCmd(ElevatorPosition.Hatch2));
        addSequential(new WaitCommand(0.5));
        addSequential(new ClimbHookCmd(true), 0.5);
        addSequential(new ClimbTeleopCmd());
    }
}