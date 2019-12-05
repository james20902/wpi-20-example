package frc.team5115.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class RamseteFollower {

    DriveTrain drivetrain;
    Trajectory trajectory;
    Heartbeat timer;
    RamseteController ramseteFeedback;
    PID leftController, rightController;

    DifferentialDriveWheelSpeeds previousSpeeds;

    public RamseteFollower(Trajectory trajectory, DriveTrain drivetrain, double P, double D){
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        timer = new Heartbeat();
        ramseteFeedback = new RamseteController();
        leftController = new PID(P, 0, D);
        rightController = new PID(P, 0, D);
    }

    public void prime(){
        timer.start();
        Trajectory.State initialState = trajectory.sample(timer.getAbsolute());
        previousSpeeds = drivetrain.driveKinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond,
                0,
                (initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond)));
        leftController.reset();
        rightController.reset();
    }

    public void execute(){
        timer.check();
        DifferentialDriveWheelSpeeds targetWheelSpeeds = drivetrain.getDriveKinematics().toWheelSpeeds(
                ramseteFeedback.calculate(drivetrain.getCurrentPose(), trajectory.sample(timer.getAbsolute())));
        DifferentialDriveWheelSpeeds currentWheelSpeeds = drivetrain.getWheelSpeeds();

        double leftTarget = targetWheelSpeeds.leftMetersPerSecond;
        double rightTarget = targetWheelSpeeds.rightMetersPerSecond;
        double leftCurrent = currentWheelSpeeds.leftMetersPerSecond;
        double rightCurrent = currentWheelSpeeds.leftMetersPerSecond;

        double leftFeedforward = drivetrain.getFeedforwardData().calculate(leftTarget,
                (leftTarget - previousSpeeds.leftMetersPerSecond) / timer.getDifference());
        double rightFeedforward = drivetrain.getFeedforwardData().calculate(leftTarget,
                (leftTarget - previousSpeeds.leftMetersPerSecond) / timer.getDifference());

        double leftOutput = leftFeedforward + leftController.calculate(leftCurrent, leftTarget);
        double rightOutput = rightFeedforward + rightController.calculate(rightCurrent, rightTarget);

        drivetrain.driveVoltage(leftOutput, rightOutput);
    }

}
