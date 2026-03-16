package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.IndexRPS;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonPresetShootCommand extends Command {
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private int shotrps;

    public AutonPresetShootCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake, int shotrps) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.intake = intake;
        this.shotrps = shotrps;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(shotrps);
        intake.intake(IndexRPS);
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
