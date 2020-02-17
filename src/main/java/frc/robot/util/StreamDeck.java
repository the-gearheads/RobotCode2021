package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class StreamDeck {
    private final int buttons;
    private final NetworkTableEntry icons;
    private final String[] iconsDefault;
    private final NetworkTable actionTable;
    private final NetworkTable statusTable;
    private NetworkTableEntry[] actions;
    private NetworkTableEntry[] statuses;

    public StreamDeck(int id, int buttons) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("/StreamDeck").getSubTable(String.valueOf(id));
        actions = new NetworkTableEntry[buttons];
        statuses = new NetworkTableEntry[buttons];
        actionTable = table.getSubTable("Action");
        statusTable = table.getSubTable("Status");
        this.icons = table.getEntry("Icons");
        this.buttons = buttons;
        iconsDefault = new String[buttons];
        Arrays.fill(iconsDefault, "default");
        this.icons.setStringArray(iconsDefault);

        reset();
    }

    public String getIcon(int index) {
        String[] iconArray = icons.getStringArray(iconsDefault);
        return iconArray[index];
    }

    public void setIcon(int index, String icon) {
        String[] iconArray = icons.getStringArray(iconsDefault);
        iconArray[index] = icon;
        icons.setStringArray(iconArray);
    }

    public void setAction(int index, boolean action) {
        actions[index].setBoolean(action);
    }

    public boolean getAction(int index) {
        return actions[index].getBoolean(false);
    }

    public void setStatus(int index, boolean status) {
        statuses[index].setBoolean(status);
    }

    public boolean getStatus(int index) {
        return statuses[index].getBoolean(false);
    }

    public void reset() {
        for (int i = 0; i < buttons; i++) {
            actions[i] = actionTable.getEntry(String.valueOf(i));
            statuses[i] = statusTable.getEntry(String.valueOf(i));
            actions[i].setBoolean(false);
            statuses[i].setBoolean(false);
        }
    }

}