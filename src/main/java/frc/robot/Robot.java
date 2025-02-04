// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.hocLib.HocRobot;
import frc.hocLib.Rio;
import frc.hocLib.Telemetry;
import frc.hocLib.Telemetry.PrintPriority;
import frc.hocLib.util.CrashTracker;
import frc.robot.algaeWheel.AlgaeWheel;
import frc.robot.algaeWheel.AlgaeWheel.AlgaeWheelConfig;
import frc.robot.auton.Auton;
import frc.robot.codriver.Codriver;
import frc.robot.codriver.Codriver.CodriverConfig;
import frc.robot.configs.CBOT_2025;
import frc.robot.configs.PBOT_2025;
import frc.robot.configs.TBOT_2025;
import frc.robot.driver.Driver;
import frc.robot.driver.Driver.DriverConfig;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.generated.TunerConstants;
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

        public SwerveConfig swerve = TunerConstants.getSwerveConfig();
        public ElevatorConfig elevator = new ElevatorConfig();
        public TusksConfig tusks = new TusksConfig();
        public JawsConfig jaws = new JawsConfig();
        public AlgaeWheelConfig algaeWheel = new AlgaeWheelConfig();

        public VisionConfig vision = new VisionConfig();
        public LightsConfig lights = new LightsConfig();

        // TODO: add configurations for different subsystems
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

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        super();
        Telemetry.start(true, true, PrintPriority.NORMAL);

        try {
            Telemetry.print("--- Robot Init Starting ---");

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
            auton = new Auton();
            visionSystem = new VisionSystem(config.vision);
            lights = new Lights(config.lights);

            // Setup Default Commands for all subsystems
            setupDefaultCommands();

            Telemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        m_robotContainer.updateGoalPoseField();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        RobotContainer.photonVisionPeriodic();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
