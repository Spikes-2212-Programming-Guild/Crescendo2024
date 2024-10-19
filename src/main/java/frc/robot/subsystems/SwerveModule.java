package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * A class which represents an MK4 swerve module, using 2 NEOs.
 */
public class SwerveModule extends DashboardedSubsystem {

    private static final int PID_SLOT = 0;

    private static final double STEERING_GEAR_RATIO = 12.8;
    private static final double DRIVING_GEAR_RATIO = 6.12;
    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_METERS * Math.PI;

    private static final double SECONDS_IN_MINUTE = 60;
    private static final double DEGREES_IN_ROTATION = 360;

    private static final int DRIVE_FREE_CURRENT_LIMIT = 60;
    private static final int DRIVE_STALL_CURRENT_LIMIT = 80;
    private static final int SPARKMAX_PERIODIC_FRAME_MS = 40;

    private final CANSparkMax driveController;
    private final CANSparkMax turnController;
    private final CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardSettings turnFeedForwardSettings;
    private final FeedForwardController driveFeedForwardController;
    private final FeedForwardController turnFeedForwardController;

    private final boolean cancoderInverted;
    private final boolean driveInverted;
    private final double cancoderOffset; //rotations

    public SwerveModule(String namespaceName, CANSparkMax driveController, CANSparkMax turnController,
                        CANcoder absoluteEncoder, boolean cancoderInverted,
                        PIDSettings drivePIDSettings, FeedForwardSettings driveFeedForwardSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings turnFeedForwardSettings, boolean driveInverted,
                        double cancoderOffset) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteEncoder = absoluteEncoder;
        this.cancoderInverted = cancoderInverted;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.turnFeedForwardSettings = turnFeedForwardSettings;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        this.turnFeedForwardController = new FeedForwardController(turnFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        this.driveEncoder = driveController.getEncoder();
        this.turnEncoder = turnController.getEncoder();
        this.driveInverted = driveInverted;
        this.cancoderOffset = cancoderOffset;
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    public void set(SwerveModuleState state, boolean usePID, boolean limitSpeed) {
        if (Math.abs(state.speedMetersPerSecond) < Drivetrain.MIN_SPEED_METERS_PER_SECONDS && limitSpeed) {
            stop();
            return;
        }
        state = optimize(state, Rotation2d.fromDegrees(getRelativeAngle()));
        double angle = state.angle.getDegrees();
        setAngle(angle);
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    public void resetRelativeTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteAngle());
    }

    public void stop() {
        driveController.stopMotor();
        turnController.stopMotor();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    //angle between 0 and 360
    public double getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue()).getDegrees();
    }

    //angle between 0 and 360
    public void setAngle(double angle) {
        configureTurnController();
        configFF();
        double kS = turnFeedForwardController.getkS();
        turnFeedForwardController.setkS(0);
        double feedForward = turnFeedForwardController.calculate(angle);
        // decides the direction for the static constant
        if (Math.abs(getAbsoluteAngle() - angle) > DEGREES_IN_ROTATION / 2) {
            kS *= Math.signum(getAbsoluteAngle() - angle);
        } else {
            kS *= -Math.signum(getAbsoluteAngle() - angle);
        }
        feedForward += kS;
        turnController.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT, feedForward,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    //speed - m/s
    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            configFF();
            double feedForward = driveFeedForwardController.calculate(speed);
            driveController.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity, PID_SLOT,
                    feedForward, SparkPIDController.ArbFFUnits.kVoltage);
        } else driveController.set(speed / Drivetrain.MAX_SPEED_METERS_PER_SECONDS);
    }

    private void configFF() {
        driveFeedForwardController.setGains(driveFeedForwardSettings.getkS(), driveFeedForwardSettings.getkV(),
                driveFeedForwardSettings.getkA(), driveFeedForwardSettings.getkG());
        turnFeedForwardController.setGains(turnFeedForwardSettings);
    }

    private void configureDriveController() {
        driveController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, SPARKMAX_PERIODIC_FRAME_MS);
        driveController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, SPARKMAX_PERIODIC_FRAME_MS);
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
        driveController.setInverted(driveInverted);
        driveController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveController.setSmartCurrentLimit(DRIVE_STALL_CURRENT_LIMIT, DRIVE_FREE_CURRENT_LIMIT);
        driveEncoder.setVelocityConversionFactor(
                ((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS) / SECONDS_IN_MINUTE);
        driveEncoder.setPositionConversionFactor((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS);
    }

    private void configureTurnController() {
        turnController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, SPARKMAX_PERIODIC_FRAME_MS);
        turnController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, SPARKMAX_PERIODIC_FRAME_MS);
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
        turnController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnEncoder.setPositionConversionFactor((1 / STEERING_GEAR_RATIO) * DEGREES_IN_ROTATION);
    }

    private void configureAbsoluteEncoder() {
        MagnetSensorConfigs config = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(cancoderOffset);
        absoluteEncoder.getConfigurator().apply(config);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for REV onboard control.
     * Credit to team #364.
     *
     * @param desiredState the desired state
     * @param currentAngle the current module angle
     */
    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private double getRelativeAngle() {
        return turnEncoder.getPosition();
    }

    /**
     * Takes the module's angle and the desired angle, and returns it in within the scope reference and 360 degrees
     * above it.
     * Credit to team #364.
     *
     * @param scopeReference current angle
     * @param newAngle       target angle
     * @return closest angle within scope
     */
    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", driveEncoder::getVelocity);
        namespace.putNumber("distance", driveEncoder::getPosition);
        namespace.putNumber("absolute angle", this::getAbsoluteAngle);
        namespace.putNumber("relative angle", this::getRelativeAngle);
        namespace.putData("reset angle", new InstantCommand(this::resetRelativeTurnEncoder).ignoringDisable(true));
        namespace.putCommand("set angle to 0", new RunCommand(() -> set(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(0)), false, false)) {
            @Override
            public void end(boolean interrupted) {
                turnController.stopMotor();
            }
        });
        namespace.putRunnable("stop", turnController::stopMotor);
        namespace.putRunnable("config turn", this::configureTurnController);
    }
}
