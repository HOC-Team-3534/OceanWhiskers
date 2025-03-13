// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.hocLib.HocRobot;
import frc.hocLib.Logging;
import frc.hocLib.Rio;
import frc.hocLib.util.CrashTracker;
import frc.hocLib.util.GeomUtil;
import frc.hocLib.util.LoggedTunableNumber;
import frc.hocLib.util.TuningCommand;
import frc.reefscape.FieldAndTags2025;
import frc.reefscape.FieldAndTags2025.ReefLevel;
import frc.robot.commands.auton.Auton;
import frc.robot.commands.auton.Auton.AutonConfig;
import frc.robot.commands.auton.AutonStep;
import frc.robot.commands.auton.DTM;
import frc.robot.commands.auton.DTM.DTMConfig;
import frc.robot.configs.CBOT_2025;
import frc.robot.configs.PBOT_2025;
import frc.robot.configs.TBOT_2025;
import frc.robot.controllers.Codriver;
import frc.robot.controllers.Codriver.CodriverConfig;
import frc.robot.controllers.Driver;
import frc.robot.controllers.Driver.DriverConfig;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorConfig;
import frc.robot.subsystems.forbar.Forbar;
import frc.robot.subsystems.forbar.Forbar.ForbarConfig;
import frc.robot.subsystems.jaws.Jaws;
import frc.robot.subsystems.jaws.Jaws.JawsConfig;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.Lights.LightsConfig;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.vision.VisionSystem;
import frc.robot.subsystems.vision.VisionSystem.VisionConfig;
import java.util.Optional;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Robot extends HocRobot {
    @Getter private static Config config;

    public static class Config {
        public DriverConfig driver = new DriverConfig();
        public CodriverConfig codriver = new CodriverConfig();

        public SwerveConfig swerve = new SwerveConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public JawsConfig jaws = new JawsConfig();

        public ForbarConfig forbar = new ForbarConfig();

        public AutonConfig auton = new AutonConfig();
        public DTMConfig dtm = new DTMConfig();
        public VisionConfig vision = new VisionConfig();
        public LightsConfig lights = new LightsConfig();

        public Time LoopPeriod = Seconds.of(0.020);
        public boolean Tuning = true;
        public boolean EnableElasticTabSwitching = false;
    }

    @Getter private static Driver driver;
    @Getter private static Codriver codriver;

    @Getter private static Swerve swerve;
    @Getter private static Elevator elevator;
    @Getter private static Jaws jaws;
    @Getter private static VisionSystem visionSystem;
    @Getter private static Lights lights;

    @Getter private static Forbar forbar;

    @Getter private static Auton auton;
    @Getter private static DTM dtm;

    public Robot() {
        super();

        try {

            Logging.setOptions(new DogLogOptions().withCaptureDs(true));

            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

            /** Set up the config */
            switch (Rio.id) {
                case CBOT_2025:
                    config = new CBOT_2025();
                    break;
                case TBOT_2025:
                    config = new TBOT_2025();
                    break;
                case PBOT_2025:
                    config = new PBOT_2025();
                    break;
                default: // SIM and UNKNOWN
                    config = new CBOT_2025();
                    break;
            }

            if (config.Tuning) {
                LoggedTunableNumber.TUNING_MODE = true;
                TuningCommand.TUNING_MODE = true;
            }

            /**
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs

            driver = new Driver(config.driver);
            codriver = new Codriver(config.codriver);

            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);
            elevator = new Elevator(config.elevator);
            Timer.delay(canInitDelay);
            jaws = new Jaws(config.jaws);
            Timer.delay(canInitDelay);
            forbar = new Forbar(config.forbar);
            Timer.delay(canInitDelay);
            visionSystem = new VisionSystem(config.vision);
            lights = new Lights(config.lights);
            auton = new Auton(config.auton);
            dtm = new DTM(config.dtm);

            Logging.log(
                    "ZeroedComponentPoses",
                    new Pose3d[] {
                        new Pose3d(),
                        new Pose3d(),
                        new Pose3d(),
                        new Pose3d(),
                        new Pose3d(),
                        new Pose3d()
                    });

            Logging.log(
                    "InitialPositionComponentPoses",
                    new Pose3d[] {
                        new Pose3d(),
                        new Pose3d(),
                        robotToLongPivot,
                        robotToShortPivot,
                        robotToCarriage,
                        robotToAlgaeArm
                    });

            percentForbarOut.initDefault(0.0);

            // Setup Default Commands for all subsystems
            setupDefaultCommands();

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        driver.resetConfig();
        codriver.resetConfig();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
    }

    public void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        setupStates();
        RobotStates.setupStates();
    }

    Pose3d fieldToRobotCAD = new Pose3d(0.0, 0.0, 0.0414, new Rotation3d());

    Pose3d robotToLongPivot =
            fieldToRobotCAD.transformBy(
                    GeomUtil.toTransform3d(new Translation3d(0.17, 0.0, 0.4425)));
    Pose3d robotToShortPivot =
            fieldToRobotCAD.transformBy(
                    GeomUtil.toTransform3d(new Translation3d(0.28375, 0.0, 0.4325)));
    Pose3d robotToCarriage =
            fieldToRobotCAD.transformBy(
                    GeomUtil.toTransform3d(new Translation3d(0.22, 0.0, 0.6525)));
    Pose3d robotToAlgaeArm =
            fieldToRobotCAD.transformBy(GeomUtil.toTransform3d(new Translation3d(0.28, 0.0, 0.38)));

    Angle shortPivotPitchOffset = Degrees.of(-16.23 - 90.0);
    Angle shortPivotPitchRange = Degrees.of(16.23 + 43.92);

    Angle longPivotPitchOffset = Degrees.of(-90);

    Distance shortPivotLength = Inches.of(9.0);
    Distance longPivotLength = Inches.of(10.5);
    Distance backPlatePivotSeparation = Inches.of(3.01);

    @RequiredArgsConstructor
    static class ForbarComponentOffsets {
        final Pose3d shortPivot, longPivot, carriage;
        final Pose3d endOfShortPivot, endOfLongPivot;
    }

    ForbarComponentOffsets calcForbarComponentPositions(double percentOutVsIn) {
        Angle changeInShortPivotPitch =
                shortPivotPitchRange.times(MathUtil.clamp(percentOutVsIn, 0.0, 1.0));

        var shortPivotOffset =
                robotToShortPivot.transformBy(
                        GeomUtil.toTransform3d(0.0, changeInShortPivotPitch, 0.0));

        var endOfShortPivot =
                shortPivotOffset
                        .transformBy(GeomUtil.toTransform3d(0.0, shortPivotPitchOffset, 0.0))
                        .transformBy(GeomUtil.toTransform3d(shortPivotLength, 0.0, 0.0));

        var endOfLongPivot =
                GeomUtil.calcIntercetionOfCirclesInXZPlane(
                                robotToLongPivot.getTranslation(),
                                longPivotLength,
                                endOfShortPivot.getTranslation(),
                                backPlatePivotSeparation)
                        .get(1);

        var toEndOfLongPivot = endOfLongPivot.minus(robotToLongPivot.getTranslation());

        var longPivotOffset =
                new Pose3d(
                        robotToLongPivot.getTranslation(),
                        new Rotation3d(
                                0.0,
                                Math.atan2(toEndOfLongPivot.getX(), toEndOfLongPivot.getZ()),
                                0.0));

        var toEndOfLongPivotFromEndofShortPivot =
                endOfLongPivot.minus(endOfShortPivot.getTranslation());

        var carriageOffset =
                new Pose3d(
                        endOfShortPivot.getTranslation(),
                        new Rotation3d(
                                0.0,
                                Units.degreesToRadians(40)
                                        + Math.atan2(
                                                toEndOfLongPivotFromEndofShortPivot.getX(),
                                                toEndOfLongPivotFromEndofShortPivot.getZ()),
                                0.0));

        return new ForbarComponentOffsets(
                shortPivotOffset,
                longPivotOffset,
                carriageOffset,
                endOfShortPivot,
                new Pose3d(endOfLongPivot, Rotation3d.kZero));
    }

    static final LoggedTunableNumber percentForbarOut =
            new LoggedTunableNumber("Forbar/PercentPositionOutTesting");

    @Override
    public void robotPeriodic() {
        try {
            Logging.log("Match Time", DriverStation.getMatchTime());

            Logging.log(
                    "Elevator Ready for Deploy", RobotStates.ElevatorReadyToDeploy.getAsBoolean());
            Logging.log("Forbar Ready for Deploy", RobotStates.ForbarReadyToDeploy.getAsBoolean());

            Logging.log("Driver Configured", getDriver().isConfigured());

            Logging.log("Swerve is Testing", RobotStates.SwerveIsTesting.getAsBoolean());

            Logging.log("Holding Coral", RobotStates.ForbarHoldingCoral.getAsBoolean());

            Logging.log("Closest Reef Tag ID", DTM.getClosestReefID().orElse(0));

            Logging.log("Go To L4 Coral", RobotStates.GoToL4Coral.getAsBoolean());

            Logging.log(
                    "Current Reef Tag Pose Estimate",
                    DTM.getClosestReefID()
                            .flatMap(tagId -> getVisionSystem().getPoseEstimateByTag(tagId))
                            .orElse(new Pose2d()));

            Logging.log(
                    "Current Auton Step Goal Pose",
                    AutonStep.getCurrentStep()
                            .map((step) -> step.getGoalPose())
                            .orElse(new Pose2d()));

            Logging.log(
                    "Components Offsets",
                    new Pose3d[] {
                        getElevator().getState().getStage1Displacement(),
                        getElevator().getState().getStage2Displacement()
                    });

            Logging.log(
                    "Scoring/ScoredCoral",
                    new Pose3d[] {
                        FieldAndTags2025.ReefBranch.A.getScoredCoral(ReefLevel.L3),
                        FieldAndTags2025.ReefBranch.A.getScoredCoral(ReefLevel.L2),
                        FieldAndTags2025.ReefBranch.D.getScoredCoral(ReefLevel.L4)
                    });

            var forbarOffsets = calcForbarComponentPositions(percentForbarOut.get());

            Logging.log(
                    "ComponentPosesManualForbarPosition",
                    new Pose3d[] {
                        new Pose3d(),
                        new Pose3d(),
                        forbarOffsets.longPivot,
                        forbarOffsets.shortPivot,
                        forbarOffsets.carriage,
                        robotToAlgaeArm
                    });

            Logging.log("EndOfLongPivot", forbarOffsets.endOfLongPivot);
            Logging.log("EndOfShortPivot", forbarOffsets.endOfShortPivot);

            CommandScheduler.getInstance().run();
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        resetCommandsAndButtons();
        super.disabledInit();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        clearCommandsAndButtons();
        auton.init();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        AutonStep.setCurrentStep(Optional.empty());
        resetCommandsAndButtons();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        resetCommandsAndButtons();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
