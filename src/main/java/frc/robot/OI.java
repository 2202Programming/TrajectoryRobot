package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);
    //resetButton is button A (id 1)
    JoystickButton resetButton = new JoystickButton(primaryJoystick, 1);
    JoystickButton runButton = new JoystickButton(primaryJoystick, 2);

    public OI() {
        // Back button zeroes the drivetrain
        // SubsystemBase random = null;
        // new JoystickButton(primaryJoystick, 7).whenPressed(
        //         new InstantCommand(() -> DriveSubsystem.resetOdometry(), random)
        // );
        //resetButton.whenPressed(new Reset(RobotContainer.getRobotDrive()));

    }

    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
}
