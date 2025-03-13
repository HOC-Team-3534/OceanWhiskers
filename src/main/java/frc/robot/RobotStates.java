package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.hocLib.dashboard.Elastic;
import frc.hocLib.util.Util;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.robot.commands.auton.Auton;
import frc.robot.commands.auton.AutonStep;
import frc.robot.commands.auton.DTM;
import frc.robot.controllers.Codriver;
import frc.robot.controllers.Driver;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.forbar.Forbar;
import frc.robot.subsystems.jaws.Jaws;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FieldUtil;
import lombok.Getter;
import lombok.Setter;

public class RobotStates {
    private static final Driver driver = Robot.getDriver();
    private static final Codriver codriver = Robot.getCodriver();

    private static final DTM dtm = Robot.getDtm();

    // SWERVE
    private static final Swerve swerve = Robot.getSwerve();

    public static final Trigger SwerveIsTesting = swerve.isTesting;
    public static final Trigger SwerveNotTesting = SwerveIsTesting.not();

    public static final Trigger RobotCentricForward =
            driver.RobotCentricForward.and(SwerveNotTesting);
    public static final Trigger RobotCentricBackward =
            driver.RobotCentricBackward.and(SwerveNotTesting);
    public static final Trigger RobotCentricLeft = driver.RobotCentricLeft.and(SwerveNotTesting);
    public static final Trigger RobotCentricRight = driver.RobotCentricRight.and(SwerveNotTesting);

    public static final Trigger SwerveQuasiasticForward =
            driver.SwerveQuasiasticForward_UDP.and(SwerveIsTesting);
    public static final Trigger SwerveQuasiasticBackward =
            driver.SwerveQuasiasticBackward_DDP.and(SwerveIsTesting);
    public static final Trigger SwerveDynamicForward =
            driver.SwerveDynamicForward_UDP.and(SwerveIsTesting);
    public static final Trigger SwerveDynamicBackward =
            driver.SwerveDynamicBackward_DDP.and(SwerveIsTesting);

    public static final Trigger SwerveMoving =
            new Trigger(() -> swerve.isMoving())
                    .debounce(0.5)
                    .onTrue(Commands.runOnce(() -> setAlignedWithReefForDeployment(false)));

    // ELEVATOR
    private static final Elevator elevator = Robot.getElevator();

    public static final Trigger ElevatorIsTesting = elevator.isTesting.and(SwerveNotTesting);

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

    public static final Trigger ElevatorBelowOrGoingToBelowJawsCanMoveHeight =
            new Trigger(
                    () ->
                            elevator.getHeight().lt(Robot.getConfig().elevator.getJaws())
                                    || elevator.getTargetHeight()
                                            .lt(Robot.getConfig().elevator.getJaws()));

    public static final Trigger ElevatorReadyToDeploy =
            new Trigger(() -> elevator.getState().isReadyToDeploy());

    // JAWS
    private static final Jaws jaws = Robot.getJaws();

    public static final Trigger JawsCanMove = ElevatorBelowOrGoingToBelowJawsCanMoveHeight.not();

    public static final Trigger JawsIn = new Trigger(() -> jaws.getState().isIn());
    public static final Trigger JawsOut = new Trigger(() -> jaws.getState().isOut());

    // FORBAR
    public static final Forbar forbar = Robot.getForbar();

    public static final Trigger ForbarHoldingCoral =
            new Trigger(() -> forbar.getState().isHoldingCoral());
    public static final Trigger ForbarReadyToDeploy = Trigger.kFalse;

    @Setter @Getter private static boolean alignedWithReefForDeployment;

    static Trigger isAutonLevel(int level) {
        return Auton.isLevel(level)
                .latchWithReset(
                        (ForbarReadyToDeploy.and(RobotStates::isAlignedWithReefForDeployment))
                                .or(Util.teleop));
    }

    public static final Trigger GoToL2Coral = codriver.GoToL2Coral_B.or(isAutonLevel(2));
    public static final Trigger GoToL3Coral = codriver.GoToL3Coral_X.or(isAutonLevel(3));
    public static final Trigger GoToL4Coral = codriver.GoToL4Coral_Y.or(isAutonLevel(4));

    public static final Trigger PreClimb = codriver.PreClimb_Select;
    public static final Trigger Climb = codriver.Climb_Start;

    public static final Trigger AutonDeployTimeoutForceDeploy =
            Util.autoMode.and(
                    () ->
                            AutonStep.getCurrentStep()
                                    .map(AutonStep::isDeployTimedOut)
                                    .orElse(false));

    public static final Trigger GoToL3Algae = codriver.GoToL3Algae_X;
    public static final Trigger GoToL2Algae = codriver.GoToL2Algae_B;

    public static final Trigger RequestReleaseAlgae = codriver.ReleaseAlgae_LT;
    public static final Trigger RequestGrabAlgae = (GoToL2Algae.or(GoToL3Algae));

    public static final Trigger RequestJawsOut = RequestGrabAlgae.or(RequestReleaseAlgae);

    @Getter @Setter private static boolean drivingAutonomously;

    public static final Trigger DrivingAutonomously =
            new Trigger(RobotStates::isDrivingAutonomously);

    public static void setupStates() {
        driver.DTMToReefLeft_LT.and(
                        () -> FieldUtil.isRobotOnOurSide(Robot.getSwerve().getState().Pose),
                        () -> !Robot.getElevator().getState().isClimbing())
                .whileTrue(dtm.dtmToReef(ReefBranch.Side.Left))
                .onTrue(Commands.runOnce(() -> selectTab("DTM Reef")))
                .onFalse(Commands.runOnce(() -> selectTab("Teleop")));
        driver.DTMToReefRight_RT.and(
                        () -> FieldUtil.isRobotOnOurSide(Robot.getSwerve().getState().Pose),
                        () -> !Robot.getElevator().getState().isClimbing())
                .whileTrue(dtm.dtmToReef(ReefBranch.Side.Right))
                .onTrue(Commands.runOnce(() -> selectTab("DTM Reef")))
                .onFalse(Commands.runOnce(() -> selectTab("Teleop")));
        driver.DTMToHumanPlayerStation_B.and(
                        () -> FieldUtil.isRobotOnOurSide(Robot.getSwerve().getState().Pose),
                        () -> !Robot.getElevator().getState().isClimbing())
                .whileTrue(dtm.dtmToHumanPlayerStation());

        Util.autoMode.onTrue(Commands.runOnce(() -> selectTab("Autonomous")));
        Util.teleop.onTrue(Commands.runOnce(() -> selectTab("Teleop")));
    }

    static void selectTab(String tabName) {
        if (Robot.getConfig().EnableElasticTabSwitching) {
            Elastic.selectTab(tabName);
        }
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
