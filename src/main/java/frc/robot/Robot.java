// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.hocLib.HocRobot;
import frc.hocLib.Rio;
import frc.hocLib.util.CrashTracker;
import frc.robot.algaeWheel.AlgaeWheel;
import frc.robot.algaeWheel.AlgaeWheel.AlgaeWheelConfig;
import frc.robot.auton.Auton;
import frc.robot.auton.Auton.AutonConfig;
import frc.robot.codriver.Codriver;
import frc.robot.codriver.Codriver.CodriverConfig;
import frc.robot.configs.CBOT_2025;
import frc.robot.configs.PBOT_2025;
import frc.robot.configs.TBOT_2025;
import frc.robot.driver.Driver;
import frc.robot.driver.Driver.DriverConfig;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.jaws.Jaws;
import frc.robot.jaws.Jaws.JawsConfig;
import frc.robot.lights.Lights;
import frc.robot.lights.Lights.LightsConfig;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.SwerveConfig;
import frc.robot.tusks.Tusks;
import frc.robot.tusks.Tusks.TusksConfig;
import frc.robot.vision.VisionSystem;
import frc.robot.vision.VisionSystem.VisionConfig;
import lombok.Getter;

public class Robot extends HocRobot {
    @Getter private static Config config;

    public static class Config {
        public DriverConfig driver = new DriverConfig();
        public CodriverConfig codriver = new CodriverConfig();

        public SwerveConfig swerve = new SwerveConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public TusksConfig tusks = new TusksConfig();
        public JawsConfig jaws = new JawsConfig();
        public AlgaeWheelConfig algaeWheel = new AlgaeWheelConfig();

        public AutonConfig auton = new AutonConfig();
        public VisionConfig vision = new VisionConfig();
        public LightsConfig lights = new LightsConfig();
    }

    @Getter private static Driver driver;
    @Getter private static Codriver codriver;

    @Getter private static Swerve swerve;
    @Getter private static Elevator elevator;
    @Getter private static Tusks tusks;
    @Getter private static Jaws jaws;
    @Getter private static AlgaeWheel algaeWheel;
    @Getter private static VisionSystem visionSystem;
    @Getter private static Lights lights;

    @Getter private static Auton auton;

    public Robot() {
        super();

        try {

            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());

            SmartDashboard.putString(
                    "Rio Serial Number", isReal() ? RobotController.getSerialNumber() : "");

            /** Set up the config */
            switch (Rio.id) {
                    // TODO: setup config for each robots tunings outside of subsystems, removing
                    // defaults where confusing
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
            tusks = new Tusks(config.tusks);
            Timer.delay(canInitDelay);
            jaws = new Jaws(config.jaws);
            Timer.delay(canInitDelay);
            algaeWheel = new AlgaeWheel(config.algaeWheel);
            Timer.delay(canInitDelay);
            visionSystem = new VisionSystem(config.vision);
            lights = new Lights(config.lights);
            auton = new Auton(config.auton);

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
            SmartDashboard.putBoolean(
                    "Elevator Ready for Deploy", RobotStates.ElevatorReadyToDeploy.getAsBoolean());
            SmartDashboard.putBoolean(
                    "Tusks Ready for Deploy", RobotStates.TusksReadyToDeploy.getAsBoolean());
            SmartDashboard.putBoolean(
                    "Tusks Holding Coral", RobotStates.HoldingCoral.getAsBoolean());
            SmartDashboard.putBoolean("Driver Configured", getDriver().isConfigured());

            SmartDashboard.putNumber(
                    "Vision Distance to Align Left Positive (In.)",
                    getVisionSystem()
                            .getDistanceToAlignLeftPositive()
                            .orElse(Meters.zero())
                            .in(Inches));

            SmartDashboard.putBoolean("Swerve Aligned", RobotStates.SwerveAligned.getAsBoolean());

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
