package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK3.driveRatios;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.IntaterCommand.IntaterMode;
import frc.robot.commands.PivotCommand.PivotMode;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandPS5Controller driver = new CommandPS5Controller(Constants.StickConstants.stickID);

    /* Drive Controls */
    private final int translationAxis =
     PS5Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS5Controller.Axis.kRightX.value;

    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS5Controller.Button.kCircle.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, PS5Controller.Button.kTriangle.value);

    // private final JoystickButton shootSpeaker = new JoystickButton(driver, PS5Controller.Button.kR2.value);
    // private final JoystickButton shootAmp = new JoystickButton(driver, PS5Controller.Button.kR1.value);

    // private final JoystickButton intakeGround = new JoystickButton(driver, PS5Controller.Button.kL1.value);
    // private final JoystickButton intakeSource = new JoystickButton(driver, PS5Controller.Button.kL2.value);

    // private final JoystickButton stopAll = new JoystickButton(driver, PS5Controller.Button.kCross.value);
    
    driver.cross(
        new IntaterCommand()
    ).whileTrue();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Intater s_Intater = new Intater();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // shootSpeaker.whileTrue(
        //     Commands.sequence(
        //         new PivotCommand(PivotMode.SPEAKER, s_Pivot, true),
        //         new IntaterCommand(IntaterMode.SPEAKERSHOOT, s_Intater, true)
        //     )
        // );

        // shootSpeaker.whileTrue(
        //     Commands.sequence(
        //     new PivotCommand(PivotMode.LEVEL, s_Pivot, true),
        //     new IntaterCommand(IntaterMode.SPEAKERSHOOT, s_Intater, true)
        //     )
        // );

        // shootAmp.whileTrue(
        //     Commands.sequence(
        //         new PivotCommand(PivotMode.AMP, s_Pivot, true),
        //         new IntaterCommand(IntaterMode.AMPSHOOT,s_Intater,true)
        //     )
        // );

        // intakeGround.whileTrue(
        //     Commands.sequence(
        //         new PivotCommand(PivotMode.GROUND_INTAKE, s_Pivot, true),
        //         new IntaterCommand(IntaterMode.INTAKE, s_Intater, true)
        //     )
        // );

        // intakeSource.whileTrue(
        //     Commands.sequence(
        //         new PivotCommand(PivotMode.SOURCE_INTAKE, s_Pivot, true),
        //         new IntaterCommand(IntaterMode.OUTTAKE, s_Intater, true)
        //     )
        // );

        // stopAll.onTrue(
        //     Commands.parallel(
        //         new IntaterCommand(IntaterMode.STOP, s_Intater, true),
        //         new PivotCommand(PivotMode.STOP, s_Pivot, true)    
        //     )
        // );

        // intakeAndShoot.whileTrue(
        //     new IntaterCommand(IntaterMode.INTAKEANDSHOOT, s_Intater, true)
        // );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
