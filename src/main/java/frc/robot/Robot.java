// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.hocLib.HocRobot;
import frc.hocLib.Logging;
import frc.hocLib.Rio;
import frc.hocLib.util.CrashTracker;
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
