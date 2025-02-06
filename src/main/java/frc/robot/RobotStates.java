package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.codriver.Codriver;
import frc.robot.driver.Driver;

public class RobotStates {
    private static final Driver driver = Robot.getDriver();
    private static final Codriver codriver = Robot.getCodriver();

    public static final Trigger GoToL1 = codriver.GoToL1_A;
    public static final Trigger GoToL2 = codriver.GoToL2_B;
    public static final Trigger GoToL3 = codriver.GoToL3_X;
    public static final Trigger GoToL4 = codriver.GoToL4_Y.or(autonL4);

    public static final Trigger ElevatorVoltageUp = codriver.ElevatorVoltageUp_UDP;
    public static final Trigger ElevatorVoltageDown = codriver.ElevatorVoltageDown_DDP;

    public static final Trigger ReadyToDeploy = new Trigger(() -> false);

    public static void setupStates() {}

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
