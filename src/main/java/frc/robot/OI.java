package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);

    public OI() {
        // Back button zeroes the drivetrain
        new JoystickButton(primaryJoystick, 7).whenPressed(
                new InstantCommand(() -> DriveSubsystem.getInstance().resetGyroscope())
        );
    }

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
}
