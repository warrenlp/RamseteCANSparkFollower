package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import javafx.geometry.Pos;

import java.util.function.BiConsumer;
import java.util.function.Supplier;


public class RamseteWrapper extends RamseteCommand {
    private final Timer m_timer = new Timer();
    private DifferentialDriveKinematics m_kinematics;
    private RamseteController m_follower;
    private Supplier<Pose2d> m_pose;
    private Trajectory m_trajectory;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    public RamseteWrapper(Trajectory trajectory,
                          Supplier<Pose2d> pose,
                          RamseteController follower,
                          DifferentialDriveKinematics kinematics,
                          BiConsumer<Double, Double> outputMetersPerSecond,
                          Subsystem... requirements) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
        m_kinematics = kinematics;
        m_follower = follower;
        m_pose = pose;
        m_trajectory = trajectory;
        m_output = outputMetersPerSecond;
    }

    @Override
    public void initialize() {
//        super.initialize();
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond,
                        0,
                        initialState.curvatureRadPerMeter
                                * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
//        super.execute();
        double curTime = m_timer.get();

        Pose2d curPose = m_pose.get();

        SmartDashboard.putNumber("Pose2d X", curPose.getTranslation().getX());
        SmartDashboard.putNumber("Pose2d Y", curPose.getTranslation().getY());
        SmartDashboard.putNumber("Pose2d Rot", curPose.getRotation().getDegrees());

        Trajectory.State curTrajState = m_trajectory.sample(curTime);

//        SmartDashboard.putNumber("Cur Traj Pose2d X", curTrajState.poseMeters.getTranslation().getX());
//        SmartDashboard.putNumber("Cur Traj Pose2d Y", curTrajState.poseMeters.getTranslation().getY());
//        SmartDashboard.putNumber("Cur Traj Pose2d Rot", curTrajState.poseMeters.getRotation().getDegrees());

        SmartDashboard.putNumber("Cur Traj Pose2d X", curTrajState.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("Cur Traj Pose2d Y", curTrajState.poseMeters.getTranslation().getY());
        SmartDashboard.putNumber("Cur Traj Pose2d Rot", curTrajState.poseMeters.getRotation().getDegrees());

        ChassisSpeeds curChassisSpeeds = m_follower.calculate(curPose, curTrajState);

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(curChassisSpeeds);

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        System.out.println("Left Target Speed: " + leftSpeedSetpoint);
        System.out.println("Right Target Speed: " + rightSpeedSetpoint);
        SmartDashboard.putNumber("Left Target Speed", leftSpeedSetpoint);
        SmartDashboard.putNumber("Right Target Speed", rightSpeedSetpoint);

        double leftOutput = leftSpeedSetpoint;
        double rightOutput = rightSpeedSetpoint;

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}
