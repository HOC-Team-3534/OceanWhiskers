package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.hocLib.util.CachedSendableChooser;
import frc.reefscape.FieldAndTags2025.ReefBranch;
import frc.reefscape.FieldAndTags2025.SideOfField;

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

    public static void lockChoosers() {
        sideOfFieldChooser.lock();
        firstBranchChooser.lock();
        secondBranchChooser.lock();
        thirdBranchChooser.lock();
        firstBranchLevelChooser.lock();
        secondBranchLevelChooser.lock();
        thirdBranchLevelChooser.lock();
    }

    public static void unlockChoosers() {
        sideOfFieldChooser.unlock();
        firstBranchChooser.unlock();
        secondBranchChooser.unlock();
        thirdBranchChooser.unlock();
        firstBranchLevelChooser.unlock();
        secondBranchLevelChooser.unlock();
        thirdBranchLevelChooser.unlock();
    }

    public static boolean isLeftSideOfFieldSelected() {
        return sideOfFieldChooser.getSelected().equals(SideOfField.Left);
    }

    public static ReefBranch getFirstBranch() {
        return firstBranchChooser.getSelected();
    }

    public static ReefBranch getSecondBranch() {
        return secondBranchChooser.getSelected();
    }

    public static ReefBranch getThirdBranch() {
        return thirdBranchChooser.getSelected();
    }

    public static int getFirstBranchLevel() {
        return firstBranchLevelChooser.getSelected();
    }

    public static int getSecondBranchLevel() {
        return secondBranchLevelChooser.getSelected();
    }

    public static int getThirdBranchLevel() {
        return thirdBranchLevelChooser.getSelected();
    }
}
