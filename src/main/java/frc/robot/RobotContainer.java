/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import static edu.wpi.first.wpilibj.XboxController.Button;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.DRIVER_CONTROLLER_PORT;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems
    private final DriveSubsystem robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> robotDrive.arcadeDrive(
                                driverController.getY(GenericHID.Hand.kLeft),
                                driverController.getX(GenericHID.Hand.kRight)), robotDrive));

    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then
     * passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Drive at half speed when the right bumper is held
        new JoystickButton(driverController, Button.kBumperRight.value)
                .whenPressed(() -> robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> robotDrive.setMaxOutput(1));

        SmartDashboard.putData("Reset NavX", new InstantCommand(
                () -> robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0))),
                robotDrive));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(S_VOLTS,
                                V_VOLT_SECONDS_PER_METER,
                                A_VOLT_SECONDS_SQUARED_PER_METER),
                        DRIVE_KINEMATICS,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(MAX_SPEED_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DRIVE_KINEMATICS)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.5, 0)
                ),
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass config
                config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                robotDrive::getPose,
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                DRIVE_KINEMATICS,
                robotDrive::tankDrive,
                robotDrive
        );

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> robotDrive.tankDrive(0, 0));
    }
}
