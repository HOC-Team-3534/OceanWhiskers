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

    public static class SwerveRelated {
        private static final Swerve swerve = Robot.getSwerve();

        public static final Trigger isTesting = new Trigger(swerve::isTesting);

        public static final Trigger QuasiasticForward =
                driver.SwerveQuasiasticForward_UDP.and(isTesting);
        public static final Trigger QuasiasticBackward =
                driver.SwerveQuasiasticBackward_DDP.and(isTesting);
        public static final Trigger DynamicForward = driver.SwerveDynamicForward_UDP.and(isTesting);
        public static final Trigger DynamicBackward =
                driver.SwerveDynamicBackward_DDP.and(isTesting);

        public static final Trigger Aligned = new Trigger(swerve::isAligned);
    }

    public static class ElevatorRelated {
        private static final Elevator elevator = Robot.getElevator();

        public static final Trigger isTesting =
                new Trigger(elevator::isTesting).and(SwerveRelated.isTesting.not());

        public static final Trigger QuasiasticUp = codriver.QuasiasticUp_UDP.and(isTesting);
        public static final Trigger QuasiasticDown = codriver.QuasiasticDown_DDP.and(isTesting);
        public static final Trigger DynamicUp = codriver.DynamicUp_UDP.and(isTesting);
        public static final Trigger DynamicDown = codriver.DynamicDown_DDP.and(isTesting);

        public static final Trigger VoltageUp = codriver.VoltageUp_UDP.and(isTesting);
        public static final Trigger VoltageDown = codriver.VoltageDown_DDP.and(isTesting);

        public static final Trigger BelowOrGoingToBelow4In =
                new Trigger(
                        () ->
                                elevator.getHeight().lt(Inches.of(4.0))
                                        || elevator.getTargetHeight().lt(Inches.of(4.0)));

        public static final Trigger ReadyToDeploy =
                new Trigger(() -> elevator.getState().isReadyToDeploy());
    }

    public static class TusksRelated {
        private static final Tusks tusks = Robot.getTusks();

        public static final Trigger isTesting =
                new Trigger(tusks::isTesting)
                        .and(SwerveRelated.isTesting.not(), ElevatorRelated.isTesting.not());

        public static final Trigger QuasiasticUp = codriver.QuasiasticUp_UDP.and(isTesting);
        public static final Trigger QuasiasticDown = codriver.QuasiasticDown_DDP.and(isTesting);
        public static final Trigger DynamicUp = codriver.DynamicUp_UDP.and(isTesting);
        public static final Trigger DynamicDown = codriver.DynamicDown_DDP.and(isTesting);

        public static final Trigger VoltageUp = codriver.VoltageUp_UDP.and(isTesting);
        public static final Trigger VoltageDown = codriver.VoltageDown_DDP.and(isTesting);

        public static final Trigger HoldingCoral =
                new Trigger(() -> tusks.getState().isHoldingCoral());

        public static final Trigger ReadyToDeploy =
                new Trigger(() -> tusks.getState().isReadyToDeploy());
    }

    public static class AlgaeWheelRelated {
        private static final AlgaeWheel algaeWheel = Robot.getAlgaeWheel();
        public static final Trigger HoldingAlgae =
                new Trigger(() -> algaeWheel.getState().isHoldingBall());
    }

    public static class JawsRelated {
        private static final Jaws jaws = Robot.getJaws();

        public static final Trigger CanMove = ElevatorRelated.BelowOrGoingToBelow4In.not();

        public static final Trigger Opened = new Trigger(() -> jaws.getState().isOpened());
        public static final Trigger Closed = new Trigger(() -> jaws.getState().isClosed());
    }

    public static final Trigger HoldingCoral = TusksRelated.HoldingCoral;
    public static final Trigger HoldingAlgae = AlgaeWheelRelated.HoldingAlgae;

    static Trigger isAutonLevel(int level) {
        return Auton.isLevel(level).latchWithReset(HoldingCoral.not().or(Util.teleop));
    }

    public static final Trigger GoToL1Coral = codriver.GoToL1_A.or(isAutonLevel(1));
    public static final Trigger GoToL2Coral = codriver.GoToL2_B.or(isAutonLevel(2));
    public static final Trigger GoToL3Coral = codriver.GoToL3_X.or(isAutonLevel(3));
    public static final Trigger GoToL4Coral = codriver.GoToL4_Y.or(isAutonLevel(4));

    static Trigger isAutonTusksPickupSide(Tusks.Side side) {
        return Auton.isTusksSide(side).latchWithReset(HoldingCoral.or(Util.teleop));
    }

    public static final Trigger PickupCoralLeft =
            codriver.PickupCoralLeft_LT.or(isAutonTusksPickupSide(Tusks.Side.Left));
    public static final Trigger PickupCoralRight =
            codriver.PickupCoralRight_RT.or(isAutonTusksPickupSide(Tusks.Side.Right));

    public static final Trigger Deploy =
            (ElevatorRelated.ReadyToDeploy.and(TusksRelated.ReadyToDeploy))
                    .and(codriver.Deploy_LS.or(SwerveRelated.Aligned));

    public static final Trigger GoToL3Algae = Trigger.kFalse;
    public static final Trigger GoToL2Algae = Trigger.kFalse;
    public static final Trigger GoToPreClimb = Trigger.kFalse;

    public static final Trigger RequestReleaseAlgae = codriver.ReleaseAlgae_LT;
    public static final Trigger HoldAlgae = HoldingAlgae.and(RequestReleaseAlgae.not());
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
