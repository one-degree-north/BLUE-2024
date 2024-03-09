package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final CommandPS5Controller mainController = new CommandPS5Controller(Constants.StickConstants.stickID);

    /* Drive Controls */

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Intater s_Intater = new Intater();

    private boolean isRobotCentric;

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("Intake",Commands.sequence(
                new PivotCommand(PivotMode.GROUND_INTAKE, s_Pivot),
                new IntaterCommand(IntaterMode.INTAKE, s_Intater)
            )
        );
        NamedCommands.registerCommand("Shoot", 
            Commands.sequence(
                new PivotCommand(PivotMode.SPEAKER, s_Pivot),
                new IntaterCommand(IntaterMode.SPEAKERSHOOT, s_Intater)
            ).raceWith(
                Commands.waitSeconds(5)
            )
        );

        isRobotCentric = false;

        autoChooser = AutoBuilder.buildAutoChooser("FrontAuto");

        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -mainController.getLeftY(),
        //         () -> -mainController.getLeftX(),
        //         () -> -mainController.getRightX(),
        //         mainController.square()
        //     )
        // );

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> isRobotCentric
            )
        );

        s_Pivot.setDefaultCommand(new RepeatCommand(new PivotCommand(PivotMode.DEFAULT, s_Pivot)));

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        mainController.L2().whileTrue(
            Commands.sequence(
                new PivotCommand(PivotMode.GROUND_INTAKE, s_Pivot),
                new IntaterCommand(IntaterMode.INTAKE, s_Intater)
            )
        );

        mainController.R1().whileTrue(
            Commands.sequence(
                new PivotCommand(PivotMode.DEFAULT, s_Pivot),
                Commands.parallel(new IntaterCommand(IntaterMode.AMP, s_Intater),
                new PivotCommand(PivotMode.AMP, s_Pivot))
            )
        );

        mainController.R2().whileTrue(
            Commands.sequence(
                new PivotCommand(PivotMode.SPEAKER, s_Pivot),
                new IntaterCommand(IntaterMode.SPEAKERSHOOT, s_Intater)
            )
        );

        mainController.L1().whileTrue(
            Commands.sequence(
                new PivotCommand(PivotMode.SOURCE_INTAKE, s_Pivot),
                new IntaterCommand(IntaterMode.AMPINTAKE, s_Intater)
            )
        );

        mainController.povUp().whileTrue(
            Commands.sequence(
                new IntaterCommand(IntaterMode.INTAKE, s_Intater)
            )
        );

        mainController.povDown().whileTrue(
            Commands.sequence(
                new IntaterCommand(IntaterMode.OUTTAKE, s_Intater)
            )
        );

        mainController.circle().whileTrue(
            new PivotCommand(PivotMode.GROUND_INTAKE, s_Pivot)
        );

        mainController.cross().whileTrue(
            new PivotCommand(PivotMode.AMP, s_Pivot)
        );
        
        
            
        /* Driver Buttons */
        mainController.triangle().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        mainController.square().onTrue(new InstantCommand(() -> isRobotCentric = !isRobotCentric));

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
