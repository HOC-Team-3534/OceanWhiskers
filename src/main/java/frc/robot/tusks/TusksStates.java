package frc.robot.tusks;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotStates.*;
import static frc.robot.RobotStates.TusksRelated.*;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

public class TusksStates {
    private static Tusks tusks = Robot.getTusks();

    public static void setupDefaultCommand() {
        tusks.setDefaultCommand(tusks.up());
    }

    public static void setupBindings() {
        PickupCoralLeft.or(PickupCoralRight).whileTrue(tusks.pickup());
        GoToL1Coral.or(GoToL2Coral, GoToL3Coral, GoToL4Coral)
                .and(ElevatorRelated.ReadyToDeploy.and(Deploy.not()))
                .whileTrue(tusks.preDeploy());
        Deploy.and(GoToL1Coral).onTrue(tusks.deployl1());
        Deploy.and(GoToL2Coral.or(GoToL3Coral)).onTrue(tusks.deployl2l3());
        Deploy.and(GoToL4Coral).onTrue(tusks.deployl4());

        VoltageUp.whileTrue(tusks.voltageOut(() -> Volts.of(0.75)));
        VoltageDown.whileTrue(tusks.voltageOut(() -> Volts.of(-0.75)));

        QuasiasticUp.whileTrue(tusks.sysIdQuasistatic(Direction.kForward));
        QuasiasticDown.whileTrue(tusks.sysIdQuasistatic(Direction.kReverse));
        DynamicUp.whileTrue(tusks.sysIdDynamic(Direction.kForward));
        DynamicDown.whileTrue(tusks.sysIdDynamic(Direction.kReverse));
    }
}
