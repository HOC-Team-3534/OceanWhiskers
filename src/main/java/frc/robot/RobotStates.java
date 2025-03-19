package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public static final Trigger SwerveMoving = new Trigger(() -> swerve.isMoving()).debounce(0.5);

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
    public static final Trigger ForbarHoldingCoralDebounce = ForbarHoldingCoral.debounce(0.5);
    public static final Trigger ForbarReadyToDeploy = new Trigger(() -> forbar.getState().isOut());
    public static final Trigger ForbarCloseToValidScoringLocation =
            new Trigger(() -> forbar.getState().getValidScoringLocation().isPresent())
                    .debounce(1.0);

    @Setter @Getter private static boolean alignedWithReefForDeployment;

    static Trigger isAutonLevel(int level) {
        return Auton.isLevel(level)
                .latchWithReset(
                        (ForbarReadyToDeploy.and(
                                                RobotStates::isAlignedWithReefForDeployment,
                                                () -> Robot.getDoor().getState().isOut())
                                        .debounce(0.25))
                                .or(Util.teleop, ForbarHoldingCoral.not()));
    }

    public static final Trigger GoToL1Coral = codriver.GoToL1Coral_A;
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

    public static final Trigger CanDTM =
            new Trigger(
                    () ->
                            FieldUtil.isRobotOnOurSide(Robot.getSwerve().getPose())
                                    && !Robot.getElevator().getState().isClimbing());

    public static final Trigger DTMReefLeft = driver.DTMToReefLeft_LT.and(CanDTM);
    public static final Trigger DTMReefRight = driver.DTMToReefRight_RT.and(CanDTM);
    public static final Trigger DTMReefCenter = driver.DTMToReefCenter_A.and(CanDTM);
    public static final Trigger DTMHumanPlayerStation =
            driver.DTMToHumanPlayerStation_B.and(CanDTM);

    private static boolean isAlignedForPickup(Pose2d loadingStationPose) {
        Pose2d relative = loadingStationPose.relativeTo(Robot.getSwerve().getPose());
        boolean xAligned = relative.getMeasureX().isNear(Inches.zero(), Inches.of(2.0));
        boolean yAligned = relative.getMeasureY().isNear(Inches.zero(), Inches.of(24.0));
        boolean rotationAligned = Math.abs(relative.getRotation().getDegrees()) < 3.0;
        return xAligned && yAligned && rotationAligned;
    }

    public static final Trigger AlignedForPickup =
            new Trigger(
                    () ->
                            Robot.getDtm()
                                    .finalGoalPoseInFrontOfClosestLoadingStation()
                                    .map(RobotStates::isAlignedForPickup)
                                    .orElse(false));
    public static final Trigger NotCloseToReef =
            new Trigger(
                    () -> {
                        var robot = Robot.getSwerve().getPose();
                        var closestReef = Robot.getDtm().findGoalPoseInFrontOfClosestReefSide();

                        if (closestReef.isEmpty()) return false;
                        var distance =
                                robot.getTranslation()
                                        .getDistance(closestReef.get().getTranslation());

                        return distance > 1;
                    });
    public static final Trigger CanRangeCloseToWall =
            new Trigger(
                            () ->
                                    Robot.getForbar()
                                                    .getState()
                                                    .getCANrangeDistance()
                                                    .lt(Inches.of(13))
                                            && Robot.getForbar()
                                                    .getState()
                                                    .getCANrangeStdDev()
                                                    .lt(Inches.of(0.35)))
                    .and(ForbarHoldingCoral.not(), NotCloseToReef);

    public static void setupStates() {
        DTMReefLeft.whileTrue(dtm.dtmToReef(ReefBranch.Side.Left))
                .onTrue(Commands.runOnce(() -> selectTab("DTM Reef")))
                .onFalse(Commands.runOnce(() -> selectTab("Teleop")));
        DTMReefRight.whileTrue(dtm.dtmToReef(ReefBranch.Side.Right))
                .onTrue(Commands.runOnce(() -> selectTab("DTM Reef")))
                .onFalse(Commands.runOnce(() -> selectTab("Teleop")));
        DTMReefCenter.whileTrue(
                        dtm.dtmToReef(
                                new Transform2d(Inches.of(-10), Inches.zero(), new Rotation2d())))
                .onTrue(Commands.runOnce(() -> selectTab("DTM Reef")))
                .onFalse(Commands.runOnce(() -> selectTab("Teleop")));
        DTMHumanPlayerStation.whileTrue(dtm.dtmToHumanPlayerStation());

        Util.autoMode.onTrue(Commands.runOnce(() -> selectTab("Autonomous")));
        Util.teleop.onTrue(Commands.runOnce(() -> selectTab("Teleop")));

        SwerveMoving.onTrue(Commands.runOnce(() -> setAlignedWithReefForDeployment(false)));
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
