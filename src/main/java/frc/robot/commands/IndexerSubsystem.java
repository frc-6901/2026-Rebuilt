package frc.robot.commands;

import static frc.robot.Constants.IndexerConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private final TalonFX motorIndexer = new TalonFX(IndexerMotorId);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public IndexerSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IndexerGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorIndexer.getConfigurator().apply(m_motorConfig);
    }

    // Sets the power of indexer motor.
    public void setPower(double power) {
        motorIndexer.setControl(new DutyCycleOut(power));
    }

    public void stop() {
        motorIndexer.setControl(m_request.withVelocity(0));
    }
}



