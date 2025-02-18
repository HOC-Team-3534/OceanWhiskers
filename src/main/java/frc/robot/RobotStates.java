package frc.robot;

import static frc.robot.auton.Auton.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.algaeWheel.AlgaeWheel;
import frc.robot.codriver.Codriver;
import frc.robot.driver.Driver;
import frc.robot.elevator.Elevator;
import frc.robot.swerve.Swerve;
import frc.robot.tusks.Tusks;

public class RobotStates {
    private static final Driver driver = Robot.getDriver();
    private static final Codriver codriver = Robot.getCodriver();

    private static final Swerve swerve = Robot.getSwerve();
    private static final Elevator elevator = Robot.getElevator();
    private static final AlgaeWheel algaeWheel = Robot.getAlgaeWheel();
    private static final Tusks tusks = Robot.getTusks();

    public static final Trigger isSwerveTesting = new Trigger(swerve::isTesting);

    public static final Trigger SwerveQuasiasticForward =
            driver.SwerveQuasiasticForward_UDP.and(isSwerveTesting);
    public static final Trigger SwerveQuasiasticBackward =
            driver.SwerveQuasiasticBackward_DDP.and(isSwerveTesting);
    public static final Trigger SwerveDynamicForward =
            driver.SwerveDynamicForward_UDP.and(isSwerveTesting);
    public static final Trigger SwerveDynamicBackward =
            driver.SwerveDynamicBackward_DDP.and(isSwerveTesting);

    public static final Trigger isElevatorTesting =
            new Trigger(elevator::isTesting).and(isSwerveTesting.not());
    public static final Trigger isTusksTesting =
            new Trigger(tusks::isTesting).and(isElevatorTesting.not(), isSwerveTesting.not());

    public static final Trigger HoldingCoral = new Trigger(() -> tusks.getState().isHoldingCoral());

    public static final Trigger GoToL1 =
            codriver.GoToL1_A.or(isLevel(1).latchWithReset(HoldingCoral.not()));
    public static final Trigger GoToL2 =
            codriver.GoToL2_B.or(isLevel(2).latchWithReset(HoldingCoral.not()));
    public static final Trigger GoToL3 =
            codriver.GoToL3_X.or(isLevel(3).latchWithReset(HoldingCoral.not()));
    public static final Trigger GoToL4 =
            codriver.GoToL4_Y.or(isLevel(4).latchWithReset(HoldingCoral.not()));

    public static final Trigger ElevatorVoltageUp = codriver.VoltageUp_UDP.and(isElevatorTesting);
    public static final Trigger ElevatorVoltageDown =
            codriver.VoltageDown_DDP.and(isElevatorTesting);

    public static final Trigger TusksVoltageUp = codriver.VoltageUp_UDP.and(isTusksTesting);
    public static final Trigger TusksVoltageDown = codriver.VoltageDown_DDP.and(isTusksTesting);

    public static final Trigger ElevatorQuasiasticUp =
            codriver.QuasiasticUp_UDP.and(isElevatorTesting);
    public static final Trigger ElevatorQuasiasticDown =
            codriver.QuasiasticDown_DDP.and(isElevatorTesting);
    public static final Trigger ElevatorDynamicUp = codriver.DynamicUp_UDP.and(isElevatorTesting);
    public static final Trigger ElevatorDynamicDown =
            codriver.DynamicDown_DDP.and(isElevatorTesting);

    public static final Trigger TusksQuasiasticUp = codriver.QuasiasticUp_UDP.and(isTusksTesting);
    public static final Trigger TusksQuasiasticDown =
            codriver.QuasiasticDown_DDP.and(isTusksTesting);
    public static final Trigger TusksDynamicUp = codriver.DynamicUp_UDP.and(isTusksTesting);
    public static final Trigger TusksDynamicDown = codriver.DynamicDown_DDP.and(isTusksTesting);

    public static final Trigger HoldingAlgae =
            new Trigger(() -> algaeWheel.getState().isHoldingBall());

    public static final Trigger ReleaseAlgae = codriver.ReleaseAlgae_LT;
    public static final Trigger HoldAlgae = HoldingAlgae.and(ReleaseAlgae.not());
    public static final Trigger GrabAlgae = codriver.GrabAlgae_RT.and(HoldAlgae.not());

    public static final Trigger PickupCoralLeft =
            codriver.PickupCoralLeft_LT.or(
                    isTusksSide(Tusks.Side.Left).latchWithReset(HoldingCoral));
    public static final Trigger PickupCoralRight =
            codriver.PickupCoralRight_RT.or(
                    isTusksSide(Tusks.Side.Right).latchWithReset(HoldingCoral));

    public static final Trigger ElevatorReadyToDeploy =
            new Trigger(() -> elevator.getState().isReadyToDeploy());

    public static final Trigger TusksReadyToDeploy =
            new Trigger(() -> tusks.getState().isReadyToDeploy());

    // TODO: add in check on position for auto deploy
    public static final Trigger Deploy =
            (ElevatorReadyToDeploy.and(TusksReadyToDeploy)).and(codriver.Deploy_LS);

    public static final Trigger FollowingPath =
            new Trigger(() -> PathPlannerAuto.currentPathName.isEmpty()).not();

    public static void setupStates() {
        NamedCommands.registerCommand(
                "WaitUntilCoralDeployed", new WaitUntilCommand(HoldingCoral.not()));
        NamedCommands.registerCommand("WaitUntilCoralPickedUp", new WaitUntilCommand(HoldingCoral));
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
