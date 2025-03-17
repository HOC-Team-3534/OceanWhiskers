package frc.robot.subsystems.door;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.Logging;
import frc.hocLib.mechanism.TalonSRXMechanism;
import lombok.Getter;
import lombok.Setter;

public class Door extends TalonSRXMechanism {
    @Getter
    public static class DoorConfig extends Config {
        // TODO tune values for opening and closing door
        Voltage outVoltage = Volts.of(4);
        Voltage holdInVoltage = Volts.of(-1);
        Voltage inVoltage = Volts.of(-4);
        Time timeInAndOut = Seconds.of(.5);

        public DoorConfig() {
            super("Door", 18);
        }
    }

    private DoorConfig config;
    @Getter private State state = new State();

    public Door(DoorConfig config) {
        super(config);
        this.config = config;

        if (isAttached()) {
            motor.setInverted(false);
        }
    }

    @Override
    public void periodic() {
        Logging.log("Door", this);
    }

    protected Command out() {
        return startRun(
                        () -> state.setPosition(Position.GoingOut),
                        () -> setVoltageOut(config.getOutVoltage()))
                .until(state::isInStateLongEnough)
                .andThen(
                        runOnce(
                                () -> {
                                    setVoltageOut(Volts.zero());
                                    state.setPosition(Position.Out);
                                }));
    }

    protected Command in() {
        return startRun(
                        () -> state.setPosition(Position.GoingIn),
                        () -> setVoltageOut(config.getInVoltage()))
                .until(state::isInStateLongEnough)
                .andThen(runOnce(() -> state.setPosition(Position.In)));
    }

    protected Command zero() {
        return runOnce(() -> setVoltageOut(Volts.zero()));
    }

    protected Command holdIn() {
        return runOnce(() -> setVoltageOut(config.holdInVoltage));
    }

    public enum Position {
        In,
        Out,
        GoingOut,
        GoingIn
    }

    @Getter
    @Setter
    public class State {
        Position position = Position.In;
        Timer positionTimer = new Timer();

        public void setPosition(Position position) {
            if (!this.position.equals(position)) {
                positionTimer.restart();
                this.position = position;
            }
        }

        public boolean isOut() {
            return position.equals(Position.Out);
        }

        public boolean isIn() {
            return position.equals(Position.In);
        }

        public boolean isInStateLongEnough() {
            return positionTimer.hasElapsed(config.timeInAndOut.in(Seconds));
        }
    }

    @Override
    public void setupBindings() {
        DoorStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        DoorStates.setupDefaultCommand();
    }
}
