package frc.robot.subsystems.scoring;

import frc.hocLib.HocSubsystem;
import lombok.Getter;
import lombok.Setter;

public class Scoring extends HocSubsystem {

    @Getter @Setter boolean crossedLine;

    public Scoring() {
        super(new Config("Scoring"));
    }

    @Override
    public void setupBindings() {}

    @Override
    public void setupDefaultCommand() {}
}
