package frc.robot.lights;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.hocLib.HocSubsystem;

public class Lights extends HocSubsystem {

    @SuppressWarnings("unused")
    private LightsConfig config;

    public static class LightsConfig extends HocSubsystem.Config {
        public LightsConfig() {
            super("Lights");
        }
    }

    final Spark leftLights = new Spark(0);
    final Spark rightLights = new Spark(1);

    public Lights(LightsConfig config) {
        super(config);
        this.config = config;
    }

    public Command normal() {
        return run(() -> {
                    leftLights.set(0.01); // color 1 light chase
                    rightLights.set(0.01); // color 1 light chase
                })
                .withName("Lights Normal");
    }

    public Command pickUpRight() {
        return run(() -> {
                    leftLights.set(0.99); // black/off
                    rightLights.set(0.91); // violet
                })
                .withName("Lights Pick Up Right");
    }

    public Command pickUpLeft() {
        return run(() -> {
                    leftLights.set(0.65); // orange
                    rightLights.set(0.99); // black/off
                })
                .withName("Lights Pick Up Left");
    }

    public Command drivingAutonomously() {
        return run(() -> {
                    leftLights.set(
                            0.43); // color 1 and 2 beats per minute, impacted by adj. 1 and 2
                    rightLights.set(
                            0.43); // color 1 and 2 beats per minute, impacted by adj. 1 and 2
                })
                .withName("Lights DTM");
    }

    @Override
    public void setupBindings() {
        LightsStates.setupBindings();
    }

    @Override
    public void setupDefaultCommand() {
        LightsStates.setupDefaultCommand();
    }
}
