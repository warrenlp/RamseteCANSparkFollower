/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANSparkMaxSmartVelocity;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase
{
    private final CANSparkMax leftMaster = new CANSparkMaxSmartVelocity(LEFT_MASTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMaster = new CANSparkMaxSmartVelocity(RIGHT_MASTER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    // The robot's drive
    private final DifferentialDrive drive;

    // The left-side drive encoder
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;

    // The gyro sensor
    private final AHRS gyro = new AHRS();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    private NetworkTableEntry gyroHeadingEntry;
    private NetworkTableEntry leftEncoderDistanceEntry;
    private NetworkTableEntry rightEncoderDistanceEntry;
    private NetworkTableEntry leftEncoderSpeedEntry;
    private NetworkTableEntry rightEncoderSpeedEntry;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem()
    {
        System.out.println("ENCODER_DISTANCE" + ENCODER_DISTANCE);
        System.out.println("RPM_TO_MPS" + RPM_TO_MPS);

        rightMaster.setInverted(true);

        CANSparkMax leftSlave = new CANSparkMaxSmartVelocity(LEFT_SLAVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax rightSlave = new CANSparkMaxSmartVelocity(RIGHT_SLAVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        leftEncoder.setPositionConversionFactor(ENCODER_DISTANCE);
        rightEncoder.setPositionConversionFactor(ENCODER_DISTANCE);

        drive = new DifferentialDrive(leftMaster, rightMaster);
        drive.setRightSideInverted(false);
        drive.setSafetyEnabled(false);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        // Configure ShuffleBoard
        ShuffleboardTab tab = Shuffleboard.getTab("DriveSubsystem");
        gyroHeadingEntry = tab.add("Gyro Heading", 0.0).getEntry();
        leftEncoderDistanceEntry = tab.add("Left Encoder Distance", 0.0).getEntry();
        rightEncoderDistanceEntry = tab.add("Right Encoder Distance", 0.0).getEntry();
        leftEncoderSpeedEntry = tab.add("Left Encoder Speed", 0.0).getEntry();
        rightEncoderSpeedEntry = tab.add("Right Encoder Speed", 0.0).getEntry();
    }

    @Override
    public void periodic()
    {
        // Update the odometry in the periodic block
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getPosition(), rightEncoder.getPosition());

        // Broadcast critical parameters to ShuffleBoard.
        gyroHeadingEntry.setDouble(getHeading());
        leftEncoderDistanceEntry.setDouble(leftEncoder.getPosition());
        rightEncoderDistanceEntry.setDouble(rightEncoder.getPosition());
        leftEncoderSpeedEntry.setDouble(leftEncoder.getVelocity());
        rightEncoderSpeedEntry.setDouble(rightEncoder.getVelocity());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose)
    {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void  arcadeDrive(double fwd, double rot)
    {
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(-rightVolts);
    }

    public void tankDrive(double leftRPM, double rightRPM)
    {
        SmartDashboard.putNumber("tankDrive left RPM", leftRPM);
        SmartDashboard.putNumber("tankDrive right RPM", rightRPM);

        leftMaster.set(leftRPM);
        rightMaster.set(rightRPM);
    }

    public void tankDriveMPS(double leftMPS, double rightMPS)
    {
        SmartDashboard.putNumber("tankDrive left MPS", leftMPS);
        SmartDashboard.putNumber("tankDrive right MPS", rightMPS);

        double leftRPM = leftMPS * MPS_TO_RPM;
        double rightRPM = rightMPS * MPS_TO_RPM;

        SmartDashboard.putNumber("tankDrive left RPM", leftRPM);
        SmartDashboard.putNumber("tankDrive right RPM", rightRPM);

        leftMaster.set(leftRPM);
        rightMaster.set(rightRPM);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders()
    {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance()
    {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) * 0.5;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public CANEncoder getLeftEncoder()
    {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public CANEncoder getRightEncoder()
    {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput)
    {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading()
    {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading()
    {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate()
    {
        return gyro.getRate() * (GYRO_REVERSED ? -1.0 : 1.0);
    }
}
