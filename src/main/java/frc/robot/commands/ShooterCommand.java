package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final double power = ShooterConstants.MaxPower;
    private final ShooterSubsystem shooter;

    public ShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
