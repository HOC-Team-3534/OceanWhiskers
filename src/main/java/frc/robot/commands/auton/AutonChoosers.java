package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.hocLib.util.CachedSendableChooser;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.SideOfField;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.With;

public class AutonChoosers {

    private static final CachedSendableChooser<SideOfField> sideOfFieldChooser;
    private static final CachedSendableChooser<ReefBranch> firstBranchChooser,
            secondBranchChooser,
            thirdBranchChooser;
    private static final CachedSendableChooser<Integer> firstBranchLevelChooser,
            secondBranchLevelChooser,
            thirdBranchLevelChooser;

    static {
        sideOfFieldChooser = new CachedSendableChooser<>();

        firstBranchChooser = new CachedSendableChooser<>();
        secondBranchChooser = new CachedSendableChooser<>();
        thirdBranchChooser = new CachedSendableChooser<>();

        firstBranchLevelChooser = new CachedSendableChooser<>();
        secondBranchLevelChooser = new CachedSendableChooser<>();
        thirdBranchLevelChooser = new CachedSendableChooser<>();
    }

    public static void init() {
        buildSideOfFieldChooser(sideOfFieldChooser, SideOfField.Left);

        sideOfFieldChooser.onChange(AutonChoosers::reload);

        buildLevelChooser(firstBranchLevelChooser);
        buildLevelChooser(secondBranchLevelChooser);
        buildLevelChooser(thirdBranchLevelChooser);

        SmartDashboard.putData("Auton/Side of Field", sideOfFieldChooser);

        SmartDashboard.putData("Auton/First Branch", firstBranchChooser);
        SmartDashboard.putData("Auton/Second Branch", secondBranchChooser);
        SmartDashboard.putData("Auton/Third Branch", thirdBranchChooser);

        SmartDashboard.putData("Auton/First Branch Level", firstBranchLevelChooser);
        SmartDashboard.putData("Auton/Second Branch Level", secondBranchLevelChooser);
        SmartDashboard.putData("Auton/Third Branch Level", thirdBranchLevelChooser);
    }

    public static void reload(SideOfField sideOfField) {
        buildReefBranchChooser(firstBranchChooser, sideOfField);
        buildReefBranchChooser(secondBranchChooser, sideOfField);
        buildReefBranchChooser(thirdBranchChooser, sideOfField);
    }

    private static void buildSideOfFieldChooser(
            SendableChooser<SideOfField> chooser, SideOfField defaultSideOfField) {

        var otherSideOfField = defaultSideOfField.opposite();

        chooser.clearOptions();

        chooser.setDefaultOption(defaultSideOfField.name(), defaultSideOfField);
        chooser.addOption(otherSideOfField.name(), otherSideOfField);
    }

    private static void buildReefBranchChooser(
            SendableChooser<ReefBranch> chooser, SideOfField sideOfField) {
        chooser.clearOptions();

        chooser.setDefaultOption("None", null);

        for (var branch : ReefBranch.values()) {
            var reefSideOfField = branch.getReefSide().getSideOfField();
            if (reefSideOfField.isEmpty() || reefSideOfField.get().equals(sideOfField)) {
                chooser.addOption(branch.name(), branch);
            }
        }
    }

    private static void buildLevelChooser(SendableChooser<Integer> chooser) {
        chooser.clearOptions();

        chooser.setDefaultOption("4", 4);
        chooser.addOption("3", 3);
        chooser.addOption("2", 2);
    }

    @Getter
    @Setter
    @With
    @NonNull
    @RequiredArgsConstructor
    @AllArgsConstructor
    public static class Choices {
        SideOfField sideOfField;
        ReefBranch firstBranch, secondBranch, thirdBranch;
        int firstBranchLevel, secondBranchLevel, thirdBranchLevel;

        public static Choices load() {
            var sideOfField = sideOfFieldChooser.getSelected();

            var firstBranch = firstBranchChooser.getSelected();

            var secondBranch = secondBranchChooser.getSelected();

            var thirdBranch = thirdBranchChooser.getSelected();

            var firstBranchLevel = firstBranchLevelChooser.getSelected();
            var secondBranchLevel = secondBranchLevelChooser.getSelected();
            var thirdBranchLevel = thirdBranchLevelChooser.getSelected();

            if (sideOfField == null
                    || firstBranch == null
                    || secondBranch == null
                    || thirdBranch == null) return null;

            return new Choices(
                    sideOfField,
                    firstBranch,
                    secondBranch,
                    thirdBranch,
                    firstBranchLevel,
                    secondBranchLevel,
                    thirdBranchLevel);
        }

        public boolean isLeftSideOfField() {
            return sideOfField.equals(SideOfField.Left);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof Choices) {
                var typedObj = (Choices) obj;
                return this.sideOfField.equals(typedObj.sideOfField)
                        && this.firstBranch.equals(typedObj.firstBranch)
                        && this.firstBranchLevel == typedObj.firstBranchLevel
                        && this.secondBranch.equals(typedObj.secondBranch)
                        && this.secondBranchLevel == typedObj.secondBranchLevel
                        && this.thirdBranch.equals(typedObj.thirdBranch)
                        && this.thirdBranchLevel == typedObj.thirdBranchLevel;
            }
            return false;
        }
    }
}
