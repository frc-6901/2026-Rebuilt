package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.MaxRPS;
import static frc.robot.Constants.ShooterConstants.ShootRPS;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * Shoots at a preset RPM configured at command creation time.
 * 
 * <p>
 * This command executes a shooting sequence using a predefined shooter speed
 * (RPM). It coordinates the shooter, kicker, and intake subsystems to perform
 * a complete shot without needing distance calculations.
 * 
 * <p>
 * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem},
 * {@link IndexerSubsystem}
 */
public class ManualShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final IndexerSubsystem indexer;

    /**
     * Constructs a ManualShootCommand with a specific shot RPM.
     *
     * @param shooter  the shooter subsystem
     * @param kicker   the kicker subsystem
     * @param intake   the intake subsystem
     * @param supplier a function providing the RPS to shoot at, at any given
     *                 instance
     */
    public ManualShootCommand(ShooterSubsystem shooter, KickerSubsystem kicker, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.indexer = indexer;

        addRequirements(shooter, kicker, indexer);
    }

    @Override
    public void execute() {
        shooter.shoot(ShootRPS);
        indexer.enable();
        kicker.kick();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
