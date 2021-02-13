package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class OI {
    /*
       Add your joysticks and buttons here
     */
    private Joystick primaryJoystick = new Joystick(0);
    //resetButton is button A (id 1)
    JoystickButton aButton = new JoystickButton(primaryJoystick, 1); //A 
    JoystickButton bButton = new JoystickButton(primaryJoystick, 2); //B
    JoystickButton xButton = new JoystickButton(primaryJoystick, 3); //x
    JoystickButton yButton = new JoystickButton(primaryJoystick, 4); //y
    JoystickButton LB = new JoystickButton(primaryJoystick, 5); //Left bumper
    JoystickButton RB = new JoystickButton(primaryJoystick, 6); //Right bumper
    

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
