package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.reefscape.FieldAndTags2025.SideOfField;
import frc.robot.subsystems.tusks.Tusks;
import java.util.Arrays;
import lombok.Getter;

public enum ReefBranch {
    // spotless:off
    A, B, C, D, E, F, G, H, I, J, K, L;
    //spotless:on

    ReefSide reefSide;

    @Getter Tusks.Side tusksSide = name().charAt(0) % 2 == 0 ? Tusks.Side.Right : Tusks.Side.Left;

    ReefSide getReefSide() {
        if (reefSide == null) {
            reefSide =
                    Arrays.stream(ReefSide.values())
                            .filter(rs -> rs.name().contains(this.name()))
                            .findFirst()
                            .orElse(null);
        }
        return reefSide;
    }

    void addToChooser(SendableChooser<ReefBranch> chooser, SideOfField sideOfField) {
        var reefSideOfField = getReefSide().getSideOfField();
        if (reefSideOfField.isEmpty() || reefSideOfField.get().equals(sideOfField)) {
            chooser.addOption(name(), this);
        }
    }
}
