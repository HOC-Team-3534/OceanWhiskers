package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
    final Spark leftLights = new Spark(0);
    final Spark rightLights = new Spark(1);

    public LightsSubsystem() {
        super();

        setDefaultCommand(normal());
    }

    public Command normal() {
        return run(() -> {
            leftLights.set(0.01); //color 1 light chase
            rightLights.set(0.01); //color 1 light chase
        });
    }

    public Command pickUpRight() {
        return run(() -> {
            leftLights.set(0.99); //black/off
            rightLights.set(0.91); //violet
        });
    }

    public Command pickUpLeft() {
        return run(() -> {
            leftLights.set(0.65); //orange 
            rightLights.set(0.99); //black/off
        });
    }

    public Command dtm() {
        return run(() -> {
            leftLights.set(0.43); //color 1 and 2 beats per minute, impacted by adj. 1 and 2
            rightLights.set(0.43); //color 1 and 2 beats per minute, impacted by adj. 1 and 2
        });
    }
}
