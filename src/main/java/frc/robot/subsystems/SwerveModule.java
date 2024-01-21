package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class SwerveModule extends DashboardedSubsystem {

    private final SpikesLogger logger = new SpikesLogger();

    private static final RootNamespace offsetNamespace = new RootNamespace("module offsets");

    public static final Supplier<Double> FRONT_LEFT_OFFSET = offsetNamespace.addConstantDouble("front left offset",
            -201.98);
    public static final Supplier<Double> FRONT_RIGHT_OFFSET = offsetNamespace.addConstantDouble("front right offset",
            0);
    public static final Supplier<Double> BACK_LEFT_OFFSET = offsetNamespace.addConstantDouble("back left offset",
            0);
    public static final Supplier<Double> BACK_RIGHT_OFFSET = offsetNamespace.addConstantDouble("back right offset",
            121.8164215); // 121.8164215

    private static final int PID_SLOT = 0;
    private static final double STEERING_GEAR_RATIO = 12.8;
    private static final double DRIVING_GEAR_RATIO = 6.12;
    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_METERS * Math.PI;

    public final CANSparkMax driveController;
    public final CANSparkMax turnController;
    private final CANcoder absoluteEncoder;
    private final double offset;
    private final FeedForwardController driveFeedForwardController;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final boolean cancoderInverted;
    private final boolean driveInverted;

    private double lastAngle;

    public SwerveModule(String namespaceName, CANSparkMax driveController, CANSparkMax turnController,
                        CANcoder absoluteEncoder, boolean cancoderInverted, double offset, FeedForwardSettings driveFeedForwardSettings,
                        PIDSettings drivePIDSettings, PIDSettings turnPIDSettings, boolean driveInverted) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteEncoder = absoluteEncoder;
        this.cancoderInverted = cancoderInverted;
        this.offset = offset;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        driveEncoder = driveController.getEncoder();
        turnEncoder = turnController.getEncoder();
        this.driveInverted = driveInverted;
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureRelativeTurnEncoder();
        lastAngle = getAbsoluteAngle();
        configureDashboard();
    }

    public double getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue()).getDegrees();
    }

    private double getRelativeAngle() {
        return turnEncoder.getPosition();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(),
                Rotation2d.fromDegrees(getAbsoluteAngle()));
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (Math.abs(state.speedMetersPerSecond) < 0.25) {
            stop();
            return;
        }
        state = optimize(state, Rotation2d.fromDegrees(getRelativeAngle()));
//        double angle = (Math.abs(state.speedMetersPerSecond) <= (DrivetrainImpl.MAX_SPEED_METERS_PER_SECONDS * 0.01))
//                ? lastAngle : state.angle.getDegrees();
        double angle = state.angle.getDegrees();
//        logger.log("speed: " + state.speedMetersPerSecond);
        logger.log("angle: " + state.angle.getDegrees());
        setAngle(angle);
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    private void configureDriveController() {
        driveController.getPIDController().setP(drivePIDSettings.getkP());
        driveController.getPIDController().setI(drivePIDSettings.getkI());
        driveController.getPIDController().setD(drivePIDSettings.getkD());
        driveController.setInverted(driveInverted);
        driveController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveEncoder.setPositionConversionFactor((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS);
        driveEncoder.setVelocityConversionFactor(
                ((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS) / 60);
    }

    private void configureTurnController() {
        turnController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 40);
        turnController.setIdleMode(CANSparkMax.IdleMode.kCoast);
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
        turnController.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private void configureAbsoluteEncoder() {
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.withMagnetOffset(offset / 360);
        config.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        config.withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);
    }

    public void configureRelativeTurnEncoder() {
        turnEncoder.setPositionConversionFactor((1 / STEERING_GEAR_RATIO) * 360);
        turnEncoder.setPosition(getAbsoluteAngle());
    }

    public void stop() {
        driveController.stopMotor();
        turnController.stopMotor();
    }

    //angle between 0 and 360
    public void setAngle(double angle) {
        configureTurnController();
        turnController.getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition, PID_SLOT);
    }

    //speed - m/s
    private void setSpeed(double speed, boolean usePID) {
        if (usePID) {
            configureDriveController();
            driveFeedForwardController.setGains(driveFeedForwardSettings.getkS(), driveFeedForwardSettings.getkV(),
                    driveFeedForwardSettings.getkA(), driveFeedForwardSettings.getkG());
            double feedForward = driveFeedForwardController.calculate(speed);
            driveController.getPIDController().setReference(speed, CANSparkMax.ControlType.kVelocity, PID_SLOT,
                    feedForward);
        } else driveController.set(speed / Drivetrain.MAX_SPEED_METERS_PER_SECONDS);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing
     * in appropriate scope for CTRE onboard control.
     * Credit to team #364
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

    public double getSpeed() {
        return driveEncoder.getVelocity();
    }

    /**
     * Takes the module's angle and the desired angle, and returns it in within the range of 0 to 360.
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
        namespace.putData("reset angle", new InstantCommand(this::configureRelativeTurnEncoder).ignoringDisable(true));
        namespace.putData("set angle to 0",
                new FunctionalCommand(() -> set(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false), () -> {
                },
                        b -> turnController.stopMotor(), () -> false));
        namespace.putData("move",
                new FunctionalCommand(() -> set(new SwerveModuleState(1.5, Rotation2d.fromDegrees(0)), false), () -> {
                },
                        b -> turnController.stopMotor(), () -> false));
        namespace.putRunnable("stop", turnController::stopMotor);
        namespace.putNumber("applied output", driveController::getAppliedOutput);
    }
}