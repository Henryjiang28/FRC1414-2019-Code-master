package frc.robot;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.commands.Climb.ClimbHookCmd;
import frc.robot.commands.Climb.EndAllClimbsCmd;
import frc.robot.commands.Climb.EndClimbCmdGroup;
import frc.robot.commands.Climb.EndHighHeelCmdGroup;
import frc.robot.commands.Climb.Final2ClimbCmdGroup;
import frc.robot.commands.Climb.FinalClimbCmdGroup;
import frc.robot.commands.Climb.HighHeelCmdGroup;
import frc.robot.commands.Climb.StartClimbCmdGroup;
import frc.robot.commands.Drivetrain.VisionCmd;
// Xbox contoller input&output
public class OI {
    //  Defining Xbox joystick Controllers
    public XboxController driveJoystick = new XboxController(0);
    public XboxController operatorJoystick = new XboxController(1);

    // Defining Buttons on Xbox controller
    public Button climbButton = new JoystickButton(operatorJoystick, 7);
    public Button endClimbButton = new JoystickButton(operatorJoystick, 8);
    public Button finalClimbButton = new JoystickButton(driveJoystick, 7);
    public Button final2ClimbButton = new JoystickButton(driveJoystick, 8);
    public Button startingButton = new JoystickButton(driveJoystick, 10); // CHANGE
    public Button highHeelButton = new JoystickButton(driveJoystick, 4); // CHANGE
    public Button endHighHeelButton = new JoystickButton(driveJoystick, 2);
    public Button visionButton = new JoystickButton(driveJoystick, 5);

    StartClimbCmdGroup startClimbCmdGroup = new StartClimbCmdGroup();
    EndClimbCmdGroup endClimbCmdGroup = new EndClimbCmdGroup();
    FinalClimbCmdGroup finalClimbCmdGroup = new FinalClimbCmdGroup();
    Final2ClimbCmdGroup final2ClimbCmdGroup = new Final2ClimbCmdGroup();
    HighHeelCmdGroup highHeelCmdGroup = new HighHeelCmdGroup(); // CHANGE
    EndAllClimbsCmd endAllClimbsCmd;
    EndHighHeelCmdGroup endHighHeelCmdGroup = new EndHighHeelCmdGroup();
    VisionCmd visionCmd = new VisionCmd();

    public OI() {
        endAllClimbsCmd = new EndAllClimbsCmd(startClimbCmdGroup, endClimbCmdGroup, finalClimbCmdGroup, final2ClimbCmdGroup, highHeelCmdGroup, endHighHeelCmdGroup);

        // Button commands, when button is pressed, start doing a group of commands.
        climbButton.whenPressed(startClimbCmdGroup);
        endClimbButton.whenPressed(endClimbCmdGroup);
        visionButton.whileHeld(visionCmd);
        finalClimbButton.whenPressed(finalClimbCmdGroup);
        final2ClimbButton.whenPressed(final2ClimbCmdGroup);
        highHeelButton.whenPressed(highHeelCmdGroup); // CHANGE
        startingButton.whenPressed(endAllClimbsCmd);
        endHighHeelButton.whenPressed(endHighHeelCmdGroup);
    }

    // deadband for joystick input
    public double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }
}