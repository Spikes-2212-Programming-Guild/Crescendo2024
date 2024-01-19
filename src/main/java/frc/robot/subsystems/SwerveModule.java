package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.*;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class represents an MK4 swerve module, using 2 NEO motors. 1 for turning and 1 for driving.
 */
public class SwerveModule extends DashboardedSubsystem {

    /**
     * The amount of times the motor needs to move in order to make a full rotation of a certain system.
     */
    private static final double TURN_GEAR_RATIO = 12.8;
    private static final double DRIVING_GEAR_RATIO = 6.12;

    private static final int PID_SLOT = 0;

    private static final double SECONDS_IN_MINUTE = 60;
    private static final double DEGREES_IN_ROTATION = 360;

    private static final double WHEEL_DIAMETER_IN_INCHES = 4;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_METERS * Math.PI;

    private static final int TURN_CONTROLLER_FRAME_PERIOD_MS = 40;

    private final CANSparkMax driveController;
    private final CANSparkMax turnController;
    private final CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final boolean driveInverted;
    private final boolean cancoderInverted;

    private final double absoluteEncoderOffset;

    private final PIDSettings drivePIDSettings;
    private final PIDSettings turnPIDSettings;
    private final FeedForwardSettings driveFeedForwardSettings;
    private final FeedForwardController driveFeedForwardController;

    public SwerveModule(String namespaceName, CANSparkMax driveController, CANSparkMax turnController,
                        CANcoder absoluteEncoder, boolean driveInverted, boolean cancoderInverted,
                        double absoluteEncoderOffset, PIDSettings drivePIDSettings,
                        PIDSettings turnPIDSettings, FeedForwardSettings driveFeedForwardSettings) {
        super(namespaceName);
        this.driveController = driveController;
        this.turnController = turnController;
        this.absoluteEncoder = absoluteEncoder;
        this.driveInverted = driveInverted;
        this.driveEncoder = driveController.getEncoder();
        this.turnEncoder = turnController.getEncoder();
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.drivePIDSettings = drivePIDSettings;
        this.turnPIDSettings = turnPIDSettings;
        this.driveFeedForwardSettings = driveFeedForwardSettings;
        this.driveFeedForwardController = new FeedForwardController(driveFeedForwardSettings,
                FeedForwardController.DEFAULT_PERIOD);
        this.cancoderInverted = cancoderInverted;
        configureDriveController();
        configureTurnController();
        configureAbsoluteEncoder();
        configureDashboard();
    }

    public void set(SwerveModuleState state, boolean usePID) {
        if (state.speedMetersPerSecond < Drivetrain.MIN_SPEED_METERS_PER_SECONDS) {
            stop();
            return;
        }
        state = optimize(state, Rotation2d.fromDegrees(getRelativeAngle()));
        double angle = state.angle.getDegrees();
        setAngle(new Rotation2d(angle));
        setSpeed(state.speedMetersPerSecond, usePID);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(),
                Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue()));
    }

    public double getRelativeAngle() {
        return turnEncoder.getPosition();
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue());
    }

    public double getPosition() {
        return driveEncoder.getPosition();
    }

    private void configureDriveController() {
        driveController.restoreFactoryDefaults();
        driveController.getPIDController().setP(turnPIDSettings.getkP());
        driveController.getPIDController().setI(turnPIDSettings.getkI());
        driveController.getPIDController().setD(turnPIDSettings.getkD());
        driveController.setInverted(driveInverted);
        driveEncoder.setPositionConversionFactor((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS);
        driveEncoder.setVelocityConversionFactor(
                ((1 / DRIVING_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS) / SECONDS_IN_MINUTE);
    }

    private void configureTurnController() {
        turnController.restoreFactoryDefaults();
        turnController.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, TURN_CONTROLLER_FRAME_PERIOD_MS);
        turnController.getPIDController().setP(turnPIDSettings.getkP());
        turnController.getPIDController().setI(turnPIDSettings.getkI());
        turnController.getPIDController().setD(turnPIDSettings.getkD());
        turnEncoder.setPositionConversionFactor((1 / TURN_GEAR_RATIO) * 360);
    }

    private void configureAbsoluteEncoder() {
        //restore the encoder to factory defaults
        absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs());
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        config.withMagnetOffset(absoluteEncoderOffset / DEGREES_IN_ROTATION);
        config.withSensorDirection(cancoderInverted ? SensorDirectionValue.Clockwise_Positive :
                SensorDirectionValue.CounterClockwise_Positive);
    }

    public void configureRelativeTurnEncoder() {
        turnEncoder.setPosition(getAbsoluteAngle().getDegrees());
    }

    public void stop() {
        driveController.stopMotor();
        turnController.stopMotor();
    }

    /**
     * In degrees.
     */
    private void setAngle(Rotation2d angle) {
        configureTurnController();
        turnController.getPIDController().setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);
    }

    private void setSpeed(double speedMetersPerSecond, boolean usePID) {
        configureDriveController();
        if (usePID) {
            driveFeedForwardController.setGains(driveFeedForwardSettings);
            double feedForward = driveFeedForwardController.calculate(speedMetersPerSecond);
            driveController.getPIDController().setReference(speedMetersPerSecond, CANSparkBase.ControlType.kVelocity,
                    PID_SLOT, feedForward);
        } else driveController.set(speedMetersPerSecond / Drivetrain.MAX_SPEED_METERS_PER_SECONDS);
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
    }
}
