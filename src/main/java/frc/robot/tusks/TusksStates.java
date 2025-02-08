package frc.robot.tusks;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

public class TusksStates {
    private static Tusks tusks = Robot.getTusks();

    public static void setupDefaultCommand() {
        tusks.setDefaultCommand(tusks.up());
    }

    public static void setupBindings() {
        PickupCoralLeft.or(PickupCoralRight).whileTrue(tusks.pickup());
        ElevatorReadyToDeploy.and(Deploy.not()).whileTrue(tusks.preDeploy());
        Deploy.onTrue(tusks.deploy());

        TusksQuasiasticUp.whileTrue(tusks.sysIdQuasistatic(Direction.kForward));
        TusksQuasiasticDown.whileTrue(tusks.sysIdQuasistatic(Direction.kReverse));
        TusksDynamicUp.whileTrue(tusks.sysIdDynamic(Direction.kForward));
        TusksDynamicDown.whileTrue(tusks.sysIdDynamic(Direction.kReverse));
    }
}
