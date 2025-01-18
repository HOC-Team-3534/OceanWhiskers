package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final int LEADER_ID = 14;
    private final int FOLLOWER_ID = 15;

    private final TalonFX elevator = new TalonFX(LEADER_ID);
    private final TalonFX follower = new TalonFX(FOLLOWER_ID);

    private final Angle MAX_HEIGHT_ANGLE = Rotations.of(10); // TODO: fix max
    private final Distance MAX_HEIGHT_LINEAR = Inches.of(53.6); // Checked with Manny. Total travel of elevator

    private final State state = new State();

    public ElevatorSubsystem() {
        super();

        follower.setControl(new Follower(LEADER_ID, true));

        elevator.setPosition(0);

        var slotConfigs = new SlotConfigs();

        slotConfigs.kP = 0.25;
        slotConfigs.kI = 0;
        slotConfigs.kD = 0;

        slotConfigs.kG = 0;
        slotConfigs.kS = 0;
        slotConfigs.kV = 0;
        slotConfigs.kA = 0;

        elevator.getConfigurator().apply(slotConfigs);

        var mmConfigs = new MotionMagicConfigs();

        mmConfigs.MotionMagicCruiseVelocity = 100;
        mmConfigs.MotionMagicAcceleration = 300;
        mmConfigs.MotionMagicJerk = 2500;

        elevator.getConfigurator().apply(mmConfigs);

        setDefaultCommand(runEnd(() -> {
            if (isAtBottomOfTravel() || state.isClimbing())
                setVoltageOutToZero();
            else
                setHeight(Inches.of(0));
        }, this::setVoltageOutToZero));
    }

    public Command raiseToHeight(Distance targetHeight) {
        return runEnd(() -> setHeight(targetHeight), this::setVoltageOutToZero);
    }

    Distance toHeight(Angle angle) {
        return angle.div(MAX_HEIGHT_ANGLE).times(MAX_HEIGHT_LINEAR);
    }

    Angle fromHeight(Distance height) {
        return height.div(MAX_HEIGHT_LINEAR).times(MAX_HEIGHT_ANGLE);
    }

    public Distance getHeight() {
        return toHeight(elevator.getPosition().getValue());
    }

    public boolean isAtBottomOfTravel() {
        return getHeight().isNear(Inches.of(0), Inches.of(1));
    }

    public void setHeight(Distance height) {
        var targetPosition = fromHeight(height);
        if (targetPosition.gt(MAX_HEIGHT_ANGLE))
            targetPosition = MAX_HEIGHT_ANGLE;
        if (targetPosition.lt(Rotations.of(0)))
            targetPosition = Rotations.of(0);
        elevator.setControl(new MotionMagicVoltage(targetPosition));
    }

    public void setVoltageOutToZero() {
        elevator.setControl(new VoltageOut(0));
    }

    public State getState() {
        return state;
    }

    public class State {
        private boolean climbing;

        public boolean isClimbing() {
            return climbing;
        }
    }
}
