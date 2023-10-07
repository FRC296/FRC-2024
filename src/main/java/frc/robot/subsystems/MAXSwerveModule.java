// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
    private final TalonFX m_drivingTalonFX;
    private final CANSparkMax m_turningSparkMax;

    private final AbsoluteEncoder m_turningEncoder;

    private final SparkMaxPIDController m_turningPIDController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingTalonFX = new TalonFX(drivingCANId);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

// Factory reset for consistency in configurations
        m_drivingTalonFX.configFactoryDefault();
        m_turningSparkMax.restoreFactoryDefaults();

// Setup encoder and PID controllers for the turning SPARKS MAX.
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
        m_turningPIDController = m_turningSparkMax.getPIDController();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

// Setup encoder for the driving TalonFX.
        m_drivingTalonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

// Apply position and velocity conversion factors for the driving encoder.
        double drivingEncoderCountsPerRevolution = 2048.0;
        m_drivingTalonFX.configSelectedFeedbackCoefficient(ModuleConstants.kDrivingEncoderPositionFactor / drivingEncoderCountsPerRevolution, 0, 10);

// Apply position and velocity conversion factors for the turning encoder.
        m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

// Set the PID gains for the driving motor.
        m_drivingTalonFX.config_kP(0, ModuleConstants.kDrivingP);
        m_drivingTalonFX.config_kI(0, ModuleConstants.kDrivingI);
        m_drivingTalonFX.config_kD(0, ModuleConstants.kDrivingD);
        m_drivingTalonFX.config_kF(0, ModuleConstants.kDrivingFF);
        m_drivingTalonFX.configNominalOutputForward(0);
        m_drivingTalonFX.configNominalOutputReverse(0);
        m_drivingTalonFX.configPeakOutputForward(ModuleConstants.kDrivingMaxOutput);
        m_drivingTalonFX.configPeakOutputReverse(ModuleConstants.kDrivingMinOutput);


        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        m_turningPIDController.setP(ModuleConstants.kTurningP);
        m_turningPIDController.setI(ModuleConstants.kTurningI);
        m_turningPIDController.setD(ModuleConstants.kTurningD);
        m_turningPIDController.setFF(ModuleConstants.kTurningFF);
        m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
                ModuleConstants.kTurningMaxOutput);

        m_drivingTalonFX.setNeutralMode(ModuleConstants.kDrivingMotorNeutralMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingTalonFX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, ModuleConstants.kDrivingMotorCurrentLimit, 0, 0));
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        // m_drivingTalonFX.configAllSettings(null);
        m_turningSparkMax.burnFlash();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingTalonFX.setSelectedSensorPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(m_drivingTalonFX.getSelectedSensorVelocity(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                m_drivingTalonFX.getSelectedSensorPosition(),
                new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingTalonFX.set(ControlMode.Velocity, optimizedDesiredState.speedMetersPerSecond);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /**
     * Zeroes all the SwerveModule encoders.
     */
    public void resetEncoders() {
        m_drivingTalonFX.setSelectedSensorPosition(0);
    }
}
