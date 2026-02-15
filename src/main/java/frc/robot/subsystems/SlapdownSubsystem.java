package frc.robot.subsystems;

import static frc.robot.Constants.SlapdownConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlapdownSubsystem extends SubsystemBase {
    private final TalonFX motorSlapdown = new TalonFX(SlapdownMotorId, "rio");
    // private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    public boolean isSlapdownDeployed = false;

    public SlapdownSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorSlapdown.getConfigurator().apply(m_motorConfig);
        motorSlapdown.setPosition(0);
    }

    public void slapdown() {
        motorSlapdown.setControl(m_request.withPosition(intakePosition));
        isSlapdownDeployed = true;
    }

    //Retracts the slapdown.
    public void retractSlapdown() {
        motorSlapdown.setControl(m_request.withPosition(homePosition));
        isSlapdownDeployed = false;
    }

    public boolean getSlapdownDeployed() {
        return isSlapdownDeployed;
    }
}
