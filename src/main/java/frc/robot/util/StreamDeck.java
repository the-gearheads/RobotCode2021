package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class StreamDeck {
    private final NetworkTableEntry icons;
    private final String[] iconsDefault;
    private NetworkTableEntry[] actions;
    private NetworkTableEntry[] statuses;

    public StreamDeck(int id, int buttons) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("/StreamDeck").getSubTable(String.valueOf(id));
        NetworkTable actionTable = table.getSubTable("Action");
        NetworkTable statusTable = table.getSubTable("Status");
        actions = new NetworkTableEntry[buttons];
        statuses = new NetworkTableEntry[buttons];
        this.icons = table.getEntry("Icons");

        iconsDefault = new String[buttons];
        Arrays.fill(iconsDefault, "default");

        for (int i = 0; i < buttons; i++) {
            actions[i] = actionTable.getEntry(String.valueOf(i));
            statuses[i] = statusTable.getEntry(String.valueOf(i));
            actions[i].setBoolean(false);
            statuses[i].setBoolean(false);
        }
        this.icons.setStringArray(iconsDefault);
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

}