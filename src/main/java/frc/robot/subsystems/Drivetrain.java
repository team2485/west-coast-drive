/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import WarlordsLib.motorcontrol.WL_SparkMax;
import WarlordsLib.sensors.SparkMaxAlternateEncoder;
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
    private WL_TalonFX m_sparkLeft2;
    private WL_TalonFX m_sparkLeft3;

    private WL_TalonFX m_sparkRight1Leader;
    private WL_TalonFX m_sparkRight2;
    private WL_TalonFX m_sparkRight3;

    private SparkMaxAlternateEncoder m_encoderLeft;
    private SparkMaxAlternateEncoder m_encoderRight;

    private PigeonIMU m_pigeon;

    public Drivetrain() {
        this.m_talonLeft1Leader = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_LEADER);
        this.m_talonLeft2 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_FOLLOWER_2);
        this.m_talonLeft3 = new WL_SparkMax(Constants.Drivetrain.SPARK_LEFT_PORT_FOLLOWER_3);

        this.m_talonRight1Leader = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_LEADER);
        this.m_talonRight2 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_FOLLOWER_2);
        this.m_talonRight3 = new WL_SparkMax(Constants.Drivetrain.SPARK_RIGHT_PORT_FOLLOWER_3);

        this.m_talonLeft1Leader.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);
        this.m_talonRight1Leader.setSmartCurrentLimit(Constants.Drivetrain.MAX_CURRENT);

        this.m_talonLeft1Leader.setFollowers(m_sparkLeft2, m_sparkLeft3);
        this.m_talonRight1Leader.setFollowers(m_sparkRight2, m_sparkRight3);


        this.m_pigeon = new PigeonIMU(Constants.Drivetrain.PIGEON_IMU_PORT);
        this.m_drive = new DifferentialDrive(m_sparkLeft1Leader, m_sparkRight1Leader);
        this.m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(m_pigeon.getFusedHeading()));

        this.m_encoderLeft = new SparkMaxAlternateEncoder(Constants.Drivetrain.LEFT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);
        this.m_encoderRight = new SparkMaxAlternateEncoder(Constants.Drivetrain.RIGHT_ENCODER_SPARK, Constants.Drivetrain.ENCODER_CPR);

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
