package frc.robot.tusks;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXArm;
import frc.hocLib.util.CachedValue;
import frc.robot.Robot;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Tusks extends TalonSRXArm {

    public static class TusksConfig extends TalonSRXArm.Config {
        @Getter private boolean motionMagicEnabled;

        @Getter private Angle up = Degrees.of(91);
        @Getter private Angle pickup = Degrees.of(42);
        @Getter private Angle preDeploy = Degrees.of(40);
        @Getter private Angle l4Deploy = Degrees.of(-45);
        @Getter private Angle l2l3Deploy = Degrees.of(-35);
        @Getter private Angle l1Deploy = Degrees.of(-30);

        // spotless:off
        //https://www.reca.lc/arm?armMass=%7B%22s%22%3A2%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A6%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22BAG%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        //spotless:on

        // tuned ff with coral without sysid, just manual kg and ks then recalc for kV and kA
        // kg = (voltageToMoveUp + voltageToMoveDown) / 2
        // ks = (voltageToMoveUp - voltageToMoveDown) / 2

        public TusksConfig() {
            super(
                    "Tusks",
                    18,
                    1440,
                    1.0,
                    Degrees.of(90),
                    true,
                    new ArmSlotConfig(0.44, 0.16, 0.02, 0.0, 0.00, 0.87),
                    new ArmSlotConfig(0.44, 0.75, 0.02, 0.0, 0.00, 0.87));

            setMMConfigs(RotationsPerSecond.of(1.5), RotationsPerSecondPerSecond.of(8.0), 2);
        }

        public TusksConfig enableMotionMagic() {
            this.motionMagicEnabled = true;
            return this;
        }
    }

    private SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(null, Volts.of(2), null),
                    new SysIdRoutine.Mechanism(this::setVoltageOut, this::logMotor, this));

    final State state = new State();

    private TusksConfig config;

    public Tusks(TusksConfig config) {
        super(config);
        this.config = config;

        SmartDashboard.putNumber("Tusks/Voltage Up Command", 0.75);
        SmartDashboard.putNumber("Tusks/Voltage Down Command", -0.75);

        SmartDashboard.putBoolean("Tusks Has Coral Override", false);
    }

    private Timer stillTimer = new Timer();

    @Override
    public void periodic() {
        if (getClosedLoopTarget().gt(Degrees.of(80))
                && getVelocity().lt(DegreesPerSecond.of(0.5))) {
            if (stillTimer.hasElapsed(0.25)) {
                if (getPosition().lt(Degrees.of(88.0))) {
                    state.setHoldingCoral(true);
                } else {
                    state.setHoldingCoral(false);
                }
            }
        } else {
            stillTimer.restart();
        }

        Logging.log("Tusks", this);
    }

    public enum Side {
        Left,
        Right
    }

    private void logMotor(SysIdRoutineLog log) {
        log.motor("tusks")
                .voltage(getVoltage())
                .angularPosition(getPosition())
                .angularVelocity(getVelocity());
    }

    public Command up() {
        // TODO: consider continuing to drive the motor a little to hold up
        // TODO: could reset position to 90 if greater than 90
        return goToAngle(config.up);
    }

    private Trigger RobotIsStill =
            new Trigger(() -> Robot.getSwerve().getRobotDriveDirection().isEmpty()).debounce(0.25);

    private boolean isNearPositionAndStill(Angle angle) {
        return getPosition().isNear(angle, Degrees.of(1.5))
                && Math.abs(getVelocity().in(DegreesPerSecond)) < 0.25;
    }

    private Trigger ReadyToDetectPickup =
            new Trigger(() -> isNearPositionAndStill(config.pickup))
                    .debounce(0.1)
                    .and(RobotIsStill);

    public Command pickup() {
        return goToAngle(config.pickup)
                .until(() -> ReadyToDetectPickup.getAsBoolean())
                .andThen(
                        voltageOut(
                                        () -> {
                                            if (getVelocity().lte(DegreesPerSecond.of(-7.5))
                                                    && RobotIsStill.getAsBoolean()) {
                                                state.setHoldingCoral(true);
                                            }

                                            return Volts.of(calculateArbitaryFeedforwardVolts(-1));
                                        })
                                .until(state::isHoldingCoral)
                                .andThen(goToAngle(config.pickup)))
                .withName("Tusks.Pickup");
    }

    public Command preDeploy() {
        return goToAngle(config.preDeploy);
    }

    public Command deployl4() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l4Deploy, Degrees.of(10.0)))
                        .andThen(Commands.waitSeconds(0.25)),
                goToAngle(config.l4Deploy),
                Commands.startEnd(() -> {}, () -> state.setHoldingCoral(false)));
    }

    public Command deployl2l3() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l2l3Deploy, Degrees.of(10.0)))
                        .andThen(Commands.waitSeconds(0.25)),
                goToAngle(config.l2l3Deploy),
                Commands.startEnd(() -> {}, () -> state.setHoldingCoral(false)));
    }

    public Command deployl1() {
        return Commands.deadline(
                Commands.waitUntil(() -> getPosition().isNear(config.l1Deploy, Degrees.of(10.0)))
                        .andThen(Commands.waitSeconds(0.5)),
                goToAngle(config.l1Deploy),
                Commands.startEnd(() -> {}, () -> state.setHoldingCoral(false)));
    }

    Command voltageOut(Supplier<Voltage> voltsSupplier) {
        return run(() -> setVoltageOut(voltsSupplier.get()));
    }

    Command goToAngle(Angle angle) {
        if (!isAttached()) return run(() -> {});

        if (!config.isMotionMagicEnabled()) return run(() -> setVoltageOut(Volts.of(0.5)));

        return run(() -> {
                    setRequestedCurrentSlot(state.isHoldingCoral() ? 1 : 0);
                    setTargetPosition(angle);
                })
                .withName("Tusks.Go To " + angle.in(Degrees) + "°");
    }

    protected void zero() {
        setVoltageOut(Volts.zero());
    }

    @Override
    protected void setVoltageOut(Voltage voltage) {
        if (getPosition().gt(Degrees.of(90))) voltage = Volts.of(Math.min(voltage.in(Volts), 0.0));
        if (getPosition().lt(Degrees.of(-75))) voltage = Volts.of(Math.max(voltage.in(Volts), 0.0));
        super.setVoltageOut(voltage);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public State getState() {
        return state;
    }

    public class State {
        @Setter private boolean holdingCoral;

        private CachedValue<Boolean> cachedSimulationHoldingCoralOverride =
                new CachedValue<>(
                        () -> SmartDashboard.getBoolean("Tusks Has Coral Override", false));

        public boolean isReadyToDeploy() {
            return getPosition().minus(Degrees.of(6.0)).lt(config.getPreDeploy());
        }

        public boolean isHoldingCoral() {
            return Robot.isReal() ? holdingCoral : cachedSimulationHoldingCoralOverride.get();
        }
    }

    @Override
    public void setupBindings() {
        TusksStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        TusksStates.setupDefaultCommand();
    }
}
