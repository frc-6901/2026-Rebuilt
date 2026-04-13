package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.GameConstants;

public class DriveToTarget extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Supplier<SwerveRequest.FieldCentric> driverInput;

        // null = use driver input for that axis
        private final Supplier<Double> targetX;
        private final Supplier<Double> targetY;
        private final Supplier<Angle> targetTheta;

        // PID controllers for each axis, only used if the axis is being controlled by
        // the command
        private final PIDController xController = new PIDController(1.0, 0.0, 0.0);
        private final PIDController yController = new PIDController(1.0, 0.0, 0.0);
        private final PIDController thetaController = new PIDController(1.0, 0.0, 0.0);

        private Angle thetaError;
        private Distance xError;
        private Distance yError;

        private final DoublePublisher thetaErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (theta)")
                        .publish();

        private final DoublePublisher xErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (x)")
                        .publish();

        private final DoublePublisher yErrorPub = NetworkTableInstance.getDefault()
                        .getTable("DriveToTarget")
                        .getDoubleTopic("Error (y)")
                        .publish();

        public DriveToTarget(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInput,
                        Optional<Supplier<Double>> targetX,
                        Optional<Supplier<Double>> targetY,
                        Optional<Supplier<Angle>> targetTheta) {
                this.drivetrain = drivetrain;
                this.driverInput = driverInput;

                this.targetX = targetX.orElse(null);
                this.targetY = targetY.orElse(null);
                this.targetTheta = targetTheta.orElse(null);

                addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
                xController.reset();
                yController.reset();
                thetaController.reset();
        }

        @Override
        public void execute() {
                Pose2d currentPose = drivetrain.getPose();
                FieldCentric input = driverInput.get();

                double vX = targetX != null ? xController.calculate(currentPose.getX(), targetX.get())
                                : input.VelocityX;

                double vY = targetY != null ? yController.calculate(currentPose.getY(), targetY.get())
                                : input.VelocityY;

                double omega = targetTheta != null
                                ? thetaController.calculate(currentPose.getRotation().getRadians(),
                                                targetTheta.get().in(Radians))
                                : input.RotationalRate;

                drivetrain.setControl(input.withVelocityX(vX).withVelocityY(vY).withRotationalRate(omega));

                xError = targetX != null ? Meters.of(targetX.get() - currentPose.getX()) : Meters.of(0);
                yError = targetY != null ? Meters.of(targetY.get() - currentPose.getY()) : Meters.of(0);
                thetaError = targetTheta != null
                                ? targetTheta.get().minus(currentPose.getRotation().getMeasure())
                                : Degrees.of(0);

                thetaErrorPub.set(thetaError.in(Degrees));
                xErrorPub.set(xError.in(Meters));
                yErrorPub.set(yError.in(Meters));
        }

        @Override
        public boolean isFinished() {
                boolean xComplete = targetX == null || xController.atSetpoint();
                boolean yComplete = targetY == null || yController.atSetpoint();
                boolean thetaComplete = targetTheta == null || thetaController.atSetpoint();

                return xComplete && yComplete && thetaComplete;
        }

        /** Rotates the robot to the alliance's hub. */
        public static Command rotateToHub(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInputSupplier) {
                return new DriveToTarget(
                                drivetrain,
                                driverInputSupplier,
                                Optional.empty(),
                                Optional.empty(),
                                Optional.of(() -> {
                                        Translation2d current = drivetrain.getPose().getTranslation();
                                        Translation2d hub = GameConstants.getHubLocation();

                                        double targetAngle = Math.atan2(
                                                        hub.getY() - current.getY(),
                                                        hub.getX() - current.getX());

                                        return Radians.of(targetAngle);
                                }));
        }

        /** Rotates the robot by 180 degrees. */
        public static Command rotateBy180(
                        CommandSwerveDrivetrain drivetrain,
                        Supplier<FieldCentric> driverInputSupplier) {
                return new DriveToTarget(
                                drivetrain,
                                driverInputSupplier,
                                Optional.empty(),
                                Optional.empty(),
                                Optional.of(() -> {
                                        return drivetrain.getPose().getRotation().getMeasure().plus(Degrees.of(180));
                                }));
        }
}