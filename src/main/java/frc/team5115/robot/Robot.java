package frc.team5115.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

import java.util.List;

public class Robot extends TimedRobot {

    DriveTrain dt;

    public Robot(){
        dt = new DriveTrain(4096, .1);
        // Create a voltage constraint to ensure we don't accelerate too fast
        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveTrain.ksVolts,
                                DriveTrain.kvVoltSecondsPerMeter,
                                DriveTrain.kaVoltSecondsSquaredPerMeter),
                        dt.driveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(DriveTrain.kMaxSpeedMetersPerSecond, DriveTrain.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(dt.driveKinematics)
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                new Pose2d(3, 0, new Rotation2d(0)),
                config
        );

    }


}


