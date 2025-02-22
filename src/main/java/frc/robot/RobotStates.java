package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.util.Util;
import frc.robot.algaeWheel.AlgaeWheel;
import frc.robot.auton.Auton;
import frc.robot.codriver.Codriver;
import frc.robot.driver.Driver;
import frc.robot.elevator.Elevator;
import frc.robot.jaws.Jaws;
import frc.robot.swerve.Swerve;
import frc.robot.tusks.Tusks;

public class RobotStates {
    private static final Driver driver = Robot.getDriver();
    private static final Codriver codriver = Robot.getCodriver();

    private static final Auton auton = Robot.getAuton();

    // SWERVE
    private static final Swerve swerve = Robot.getSwerve();

    public static final Trigger SwerveIsTesting = new Trigger(swerve::isTesting);

    public static final Trigger SwerveQuasiasticForward =
            driver.SwerveQuasiasticForward_UDP.and(SwerveIsTesting);
    public static final Trigger SwerveQuasiasticBackward =
            driver.SwerveQuasiasticBackward_DDP.and(SwerveIsTesting);
    public static final Trigger SwerveDynamicForward =
            driver.SwerveDynamicForward_UDP.and(SwerveIsTesting);
    public static final Trigger SwerveDynamicBackward =
            driver.SwerveDynamicBackward_DDP.and(SwerveIsTesting);

    public static final Trigger SwerveAligned = new Trigger(swerve::isAligned);

    // ELEVATOR
    private static final Elevator elevator = Robot.getElevator();

    public static final Trigger ElevatorIsTesting =
            new Trigger(elevator::isTesting).and(SwerveIsTesting.not());

    public static final Trigger ElevatorQuasiasticUp =
            codriver.QuasiasticUp_UDP.and(ElevatorIsTesting);
    public static final Trigger ElevatorQuasiasticDown =
            codriver.QuasiasticDown_DDP.and(ElevatorIsTesting);
    public static final Trigger ElevatorDynamicUp = codriver.DynamicUp_UDP.and(ElevatorIsTesting);
    public static final Trigger ElevatorDynamicDown =
            codriver.DynamicDown_DDP.and(ElevatorIsTesting);

    public static final Trigger ElevatorVoltageUp = codriver.VoltageUp_UDP.and(ElevatorIsTesting);
    public static final Trigger ElevatorVoltageDown =
            codriver.VoltageDown_DDP.and(ElevatorIsTesting);

    public static final Trigger ElevatorBelowOrGoingToBelow4In =
            new Trigger(
                    () ->
                            elevator.getHeight().lt(Inches.of(4.0))
                                    || elevator.getTargetHeight().lt(Inches.of(4.0)));

    public static final Trigger ElevatorReadyToDeploy =
            new Trigger(() -> elevator.getState().isReadyToDeploy());

    // TUSKS
    private static final Tusks tusks = Robot.getTusks();

    public static final Trigger isTesting =
            new Trigger(tusks::isTesting).and(SwerveIsTesting.not(), ElevatorIsTesting.not());

    public static final Trigger TusksQuasiasticUp = codriver.QuasiasticUp_UDP.and(isTesting);
    public static final Trigger TusksQuasiasticDown = codriver.QuasiasticDown_DDP.and(isTesting);
    public static final Trigger TusksDynamicUp = codriver.DynamicUp_UDP.and(isTesting);
    public static final Trigger TusksDynamicDown = codriver.DynamicDown_DDP.and(isTesting);

    public static final Trigger TusksVoltageUp = codriver.VoltageUp_UDP.and(isTesting);
    public static final Trigger TusksVoltageDown = codriver.VoltageDown_DDP.and(isTesting);

    public static final Trigger TusksHoldingCoral =
            new Trigger(() -> tusks.getState().isHoldingCoral());

    public static final Trigger TusksReadyToDeploy =
            new Trigger(() -> tusks.getState().isReadyToDeploy());

    // ALGAE WHEEL
    private static final AlgaeWheel algaeWheel = Robot.getAlgaeWheel();
    public static final Trigger AlgaeWheelHoldingAlgae =
            new Trigger(() -> algaeWheel.getState().isHoldingBall());

    // JAWS
    private static final Jaws jaws = Robot.getJaws();

    public static final Trigger JawsCanMove = ElevatorBelowOrGoingToBelow4In.not();

    public static final Trigger JawsOpened = new Trigger(() -> jaws.getState().isOpened());
    public static final Trigger JawsClosed = new Trigger(() -> jaws.getState().isClosed());

    static Trigger isAutonLevel(int level) {
        return Auton.isLevel(level).latchWithReset(TusksHoldingCoral.not().or(Util.teleop));
    }

    public static final Trigger GoToL1Coral = codriver.GoToL1_A.or(isAutonLevel(1));
    public static final Trigger GoToL2Coral = codriver.GoToL2_B.or(isAutonLevel(2));
    public static final Trigger GoToL3Coral = codriver.GoToL3_X.or(isAutonLevel(3));
    public static final Trigger GoToL4Coral = codriver.GoToL4_Y.or(isAutonLevel(4));

    static Trigger isAutonTusksPickupSide(Tusks.Side side) {
        return Auton.isTusksSide(side).latchWithReset(TusksHoldingCoral.or(Util.teleop));
    }

    public static final Trigger PickupCoralLeft =
            codriver.PickupCoralLeft_LT.or(isAutonTusksPickupSide(Tusks.Side.Left));
    public static final Trigger PickupCoralRight =
            codriver.PickupCoralRight_RT.or(isAutonTusksPickupSide(Tusks.Side.Right));

    public static final Trigger Deploy =
            (ElevatorReadyToDeploy.and(TusksReadyToDeploy))
                    .and(codriver.Deploy_LS.or(SwerveAligned));

    public static final Trigger GoToL3Algae = Trigger.kFalse;
    public static final Trigger GoToL2Algae = Trigger.kFalse;
    public static final Trigger GoToPreClimb = Trigger.kFalse;

    public static final Trigger RequestReleaseAlgae = codriver.ReleaseAlgae_LT;
    public static final Trigger HoldAlgae = AlgaeWheelHoldingAlgae.and(RequestReleaseAlgae.not());
    public static final Trigger RequestGrabAlgae =
            (GoToL2Algae.or(GoToL3Algae)).and(HoldAlgae.not());

    public static final Trigger RequestJawsClosed =
            RequestGrabAlgae.or(HoldAlgae, RequestReleaseAlgae);

    public static final Trigger FollowingPath =
            new Trigger(() -> PathPlannerAuto.currentPathName.isEmpty()).not();

    public static void setupStates() {
        driver.DTMToReef_A.whileTrue(auton.dtmToReef());
        driver.DTMToHumanPlayerStation_B.whileTrue(auton.dtmToHumanPlayerStation());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
