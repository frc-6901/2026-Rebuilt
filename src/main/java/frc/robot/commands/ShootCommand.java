package frc.robot.commands;

import java.util.Objects;

// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.KickerSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;

// /**
//  * Spins up the shooter to a priming speed to prepare it for shooting game
//  * pieces.
//  *
//  * <p>
//  * This command spins up the shooter to a priming speed and finishes when the
//  * specified timeout has elapsed. Useful for preparing the shooter before a
//  * shot.
//  * 
//  * <p>
//  * Requires: {@link ShooterSubsystem}, {@link KickerSubsystem}
//  */
// public class ShootPrimedRPSCommand extends Command {
//     private final ShooterSubsystem shooter;

//     private final Timer timer;
//     private final Time timeout;

//     /**
//      * Constructs a PrimeShooterCommand with a specific timeout.
//      *
//      * @param shooter the shooter subsystem
//      * @param kicker  the kicker subsystem
//      * @param timeout the maximum time to run the priming sequence
//      */
//     public ShootPrimedRPSCommand(ShooterSubsystem shooter, Time timeout) {
//         this.shooter = shooter;
//         this.timer = new Timer();
//         this.timeout = timeout;

//         addRequirements(shooter);
//     }

//     /**
//      * Initializes the command by starting the timer and spinning the shooter
//      * at the priming speed.
//      */
//     @Override
//     public void initialize() {
//         shooter.shoot(RotationsPerSecond.of(80));
//         shooter.shooterState = ShooterSubsystem.ShooterState.PRIMING;

//         timer.restart();
//     }

//     /**
//      * Ends the command when the timeout has been reached.
//      *
//      * @return {@code true} if the timeout has elapsed, {@code false} otherwise
//      */
//     @Override
//     public boolean isFinished() {
//         return timer.hasElapsed(timeout);
//     }
// }

import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

/**
 * Spins up the shooter to a priming speed to prepare it for shooting game
 * pieces, then executes the shooting sequence once the shooter is primed.
 */
public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            IndexerSubsystem indexer,
            Supplier<AngularVelocity> rpsSupplier,
            ShooterState primingState,
            ShooterState shootingState) {
        super(
                new RunCommand(() -> {
                    shooter.shoot(rpsSupplier.get());
                    shooter.shooterState = primingState;
                }, shooter).until(shooter.primed),
                new RunCommand(() -> {
                    if (indexer != null)
                        indexer.enable();
                    if (kicker != null)
                        kicker.kick();

                    shooter.shooterState = shootingState;
                }, shootRequirements(shooter, indexer, kicker)));
    }

    private static SubsystemBase[] shootRequirements(
            ShooterSubsystem shooter,
            IndexerSubsystem indexer,
            KickerSubsystem kicker) {
        return Stream.of(shooter, indexer, kicker)
                .filter(Objects::nonNull)
                .toArray(SubsystemBase[]::new);
    }
}