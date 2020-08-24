package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class OuttakeHatchCmdGroup extends CommandGroup {
    public OuttakeHatchCmdGroup() {
        addParallel(new Hatch2Cmd(true));
        addSequential(new HatchCmd(false));
    }
}