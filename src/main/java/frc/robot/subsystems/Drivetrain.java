/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import WarlordsLib.motorcontrol.WL_TalonFX;
import WarlordsLib.sensors.TalonEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase  {

    private DifferentialDrive m_drive;
    public static DifferentialDriveOdometry m_odometry;

    private WL_TalonFX m_talonLeft1Leader;
    private WL_TalonFX m_talonLeft2;
    private WL_TalonFX m_talonLeft3;

    private WL_TalonFX m_talonRight1Leader;
    private WL_TalonFX m_talonRight2;
    private WL_TalonFX m_talonRight3;

    private TalonEncoder m_encoderLeft;
    private TalonEncoder m_encoderRight;

    private PigeonIMU m_pigeon;

    public Drivetrain() {
        this.m_talonLeft1Leader = new WL_TalonFX(Constants.Drivetrain.TALON_LEFT_PORT_LEADER);
        this.m_talonLeft2 = new WL_TalonFX(Constants.Drivetrain.TALON_LEFT_PORT_FOLLOWER_2);
        this.m_talonLeft3 = new WL_TalonFX(Constants.Drivetrain.TALON_LEFT_PORT_FOLLOWER_3);

        this.m_talonRight1Leader = new WL_TalonFX(Constants.Drivetrain.TALON_RIGHT_PORT_LEADER);
        this.m_talonRight2 = new WL_TalonFX(Constants.Drivetrain.TALON_RIGHT_PORT_FOLLOWER_2);
        this.m_talonRight3 = new WL_TalonFX(Constants.Drivetrain.TALON_RIGHT_PORT_FOLLOWER_3);

        this.m_talonLeft1Leader.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);
        this.m_talonRight1Leader.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);

        this.m_talonLeft1Leader.setFollowers(m_talonLeft2, m_talonLeft3);
        this.m_talonRight1Leader.setFollowers(m_talonRight2, m_talonRight3);


        this.m_pigeon = new PigeonIMU(Constants.Drivetrain.PIGEON_IMU_PORT);
        this.m_drive = new DifferentialDrive(m_talonLeft1Leader, m_talonRight1Leader);
        this.m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));

        this.m_encoderLeft = new TalonEncoder(Constants.Drivetrain.LEFT_ENCODER_TALON, Constants.Drivetrain.ENCODER_CPR);
        this.m_encoderRight = new TalonEncoder(Constants.Drivetrain.RIGHT_ENCODER_TALON, Constants.Drivetrain.ENCODER_CPR);

        this.m_encoderRight.setInverted(true);


        this.m_encoderLeft.setDistancePerRevolution(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);
        this.m_encoderRight.setDistancePerRevolution(Constants.Drivetrain.DISTANCE_PER_REVOLUTION);


        SendableRegistry.add(this.m_drive, "DifferentialDrive");
    }


    public void curvatureDrive(double throttle, double steering, boolean isQuickTurn) {

        m_drive.curvatureDrive(throttle, steering, isQuickTurn);

    }

    /**
     * Reset encoders
     * @param posLeft left encoder position
     * @param posRight right encoder position
     */
    public void resetEncoders(double posLeft, double posRight) {
        m_encoderRight.setPosition(posLeft);
        m_encoderLeft.setPosition(posRight);
    }

    public double getLeftEncoderPosition() {
        return m_encoderLeft.getPosition();
    }

    public double getRightEncoderPosition() {
        return m_encoderRight.getPosition();
    }

    public double getLeftEncoderVelocity() {
        return m_encoderLeft.getVelocity();
    }

    public double getRightEncoderVelocity() {
        return m_encoderRight.getVelocity();
    }

    /**
     * Reset heading of gyro
     * @param heading degrees
     */
    public void setHeading(double heading) {
        m_pigeon.setFusedHeading(heading);
    }

    public double getHeading() {
        return -m_pigeon.getFusedHeading();
    }

    public void driveVolts(double leftVolts, double rightVolts) {
        m_talonLeft1Leader.setVoltage(leftVolts);
        m_talonRight1Leader.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public CANEncoder getIntakeArmEncoder() {
        return m_talonLeft3.getAlternateEncoder();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(), m_encoderRight.getVelocity());
    }

    @Override
    public void periodic() {
    }
}
