package frc.robot.subsystems.tusks;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                .and(ElevatorReadyToDeploy.and(Deploy.not()))
                .whileTrue(tusks.preDeploy());
        Deploy.and(GoToL1Coral).onTrue(tusks.deployl1());
        Deploy.and(GoToL2Coral.or(GoToL3Coral)).onTrue(tusks.deployl2l3());
        Deploy.and(GoToL4Coral).onTrue(tusks.deployl4());

        TusksVoltageUp.whileTrue(
                tusks.voltageOut(
                        () ->
                                Volts.of(
                                        SmartDashboard.getNumber(
                                                "Tusks/Voltage Up Command", 0.75))));
        TusksVoltageDown.whileTrue(
                tusks.voltageOut(
                        () ->
                                Volts.of(
                                        SmartDashboard.getNumber(
                                                "Tusks/Voltage Down Command", -0.75))));

        TusksQuasiasticUp.whileTrue(tusks.sysIdQuasistatic(Direction.kForward));
        TusksQuasiasticDown.whileTrue(tusks.sysIdQuasistatic(Direction.kReverse));
        TusksDynamicUp.whileTrue(tusks.sysIdDynamic(Direction.kForward));
        TusksDynamicDown.whileTrue(tusks.sysIdDynamic(Direction.kReverse));
    }
}
