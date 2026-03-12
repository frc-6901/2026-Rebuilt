// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IndexerConstants.*;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX motorIndex = new TalonFX(IndexMotorId, "rio");
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new Indexer. */
  public IndexerSubsystem() {
    TalonFXConfiguration m_indexerConfig = new TalonFXConfiguration();
    m_indexerConfig.Slot0 = IndexerGains;
    m_indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_indexerConfig.CurrentLimits.withStatorCurrentLimit(60);
    m_indexerConfig.Feedback.SensorToMechanismRatio = GearRatio;

    motorIndex.getConfigurator().apply(m_indexerConfig);
  }

  public void shoot() {
    motorIndex.setControl(m_request.withVelocity(IndexRPS));
  }

  public void shoot(int rps) {
    motorIndex.setControl(m_request.withVelocity(rps));
  }

  // Disables both motors by setting their power to 0.
  public void stop() {
    motorIndex.setControl(m_request.withVelocity(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
