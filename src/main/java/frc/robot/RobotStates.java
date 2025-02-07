package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.codriver.Codriver;
import frc.robot.driver.Driver;
import frc.robot.elevator.Elevator;
import frc.robot.swerve.Swerve;

public class RobotStates {
    private static final Driver driver = Robot.getDriver();
    private static final Codriver codriver = Robot.getCodriver();

    private static final Swerve swerve = Robot.getSwerve();
    public static final Trigger isSwerveTesting = new Trigger(swerve::isTesting);

    public static final Trigger CharacterizeDrive =
            driver.CharacterizeSwerve_DDP.and(isSwerveTesting);

    private static final Elevator elevator = Robot.getElevator();
    public static final Trigger isElevatorTesting = new Trigger(elevator::isTesting);

    public static final Trigger GoToL1 = codriver.GoToL1_A;
    public static final Trigger GoToL2 = codriver.GoToL2_B;
    public static final Trigger GoToL3 = codriver.GoToL3_X;
    public static final Trigger GoToL4 = codriver.GoToL4_Y.or(autonL4);

    public static final Trigger ElevatorVoltageUp =
            codriver.ElevatorVoltageUp_UDP.and(isElevatorTesting);
    public static final Trigger ElevatorVoltageDown =
            codriver.ElevatorVoltageDown_DDP.and(isElevatorTesting);

    public static final Trigger ReadyToDeploy = new Trigger(() -> false);

    public static void setupStates() {}

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
