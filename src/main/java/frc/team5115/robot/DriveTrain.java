package frc.team5115.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DriveTrain {

    //DO NOT USE THESE VALUES ON THE ROBOT, USE THE CHARACTERIZATION TOOL
    public static final double kTrackwidthMeters = 0.6;
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    WPI_TalonSRX leftController;
    WPI_TalonSRX rightController;
    WPI_TalonSRX leftSlave;
    WPI_TalonSRX rightSlave;

    AHRS gyro;

    int ticksPerRev;
    double wheelDiameterMeters;
    double metersPerTick;

    DifferentialDriveOdometry odometryCalculator;
    DifferentialDriveKinematics driveKinematics;
    SimpleMotorFeedforward feedforwardData;

    public DriveTrain(int ticksPerRev, double wheelDiameterMeters){
        leftController = new WPI_TalonSRX(1);
        rightController = new WPI_TalonSRX(2);

        leftSlave = new WPI_TalonSRX(3);
        leftSlave.set(ControlMode.Follower, 1);
        rightSlave = new WPI_TalonSRX(4);
        rightSlave.set(ControlMode.Follower, 2);

        gyro = new AHRS(SPI.Port.kMXP);

        leftController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        this.ticksPerRev = ticksPerRev;
        this.wheelDiameterMeters = wheelDiameterMeters;
        metersPerTick = (this.wheelDiameterMeters * Math.PI) / (double) this.ticksPerRev;

        odometryCalculator = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        feedforwardData = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
    }

    public void driveJoystickPercent(double x, double y, double throttle){
        double leftSpeed = Math.sqrt(3.4 * (y + x));
        double rightSpeed = Math.sqrt(3.4 * (-y + x));
        leftController.set(ControlMode.PercentOutput,  leftSpeed * throttle);
        rightController.set(ControlMode.PercentOutput, rightSpeed * throttle);
    }

    public void driveVoltage(double leftVoltage, double rightVoltage){

        leftController.set(ControlMode.PercentOutput, leftVoltage / 12.);
        rightController.set(ControlMode.PercentOutput, rightVoltage / 12.);
    }

    public void updatePosition(){
        odometryCalculator.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    }

    public Pose2d getCurrentPose(){
        return odometryCalculator.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftDistance(), getRightDistance());
    }

    public DifferentialDriveKinematics getDriveKinematics(){
        return driveKinematics;
    }

    public SimpleMotorFeedforward getFeedforwardData(){
        return feedforwardData;
    }

    public void resetHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return gyro.getYaw();
    }

    public int getLeftDistance(){
        return leftController.getSelectedSensorPosition();
    }

    public int getRightDistance(){
        return rightController.getSelectedSensorPosition();
    }
}
