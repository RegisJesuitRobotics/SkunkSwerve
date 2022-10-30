package frc.robot.utils;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

public class ListenableSendableChooser<V> implements NTSendable {
    private static final String DEFAULT = "default";
    private static final String SELECTED = "selected";
    private static final String ACTIVE = "active";
    private static final String OPTIONS = "options";
    private final Map<String, V> m_map = new LinkedHashMap<>();
    private String m_defaultChoice = "";

    private boolean hasNewValue = true;

    public ListenableSendableChooser() {
        super();
    }


    public boolean hasNewValue() {
        try {
            return hasNewValue;
        } finally {
            hasNewValue = false;
        }
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard}
     * on the desktop, the object will appear as the given name.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void addOption(String name, V object) {
        m_map.put(name, object);
    }

    /**
     * Adds the given object to the list of options and marks it as the default.
     * Functionally, this is very close to {@link #addOption(String, Object)} except
     * that it will use this as the default option if none other is explicitly
     * selected.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void setDefaultOption(String name, V object) {
        requireNonNullParam(name, "name", "setDefaultOption");
        m_defaultChoice = name;
        addOption(name, object);
    }

    /**
     * Adds the given object to the list of options and marks it as the default.
     *
     * @param name   the name of the option
     * @param object the option
     * @deprecated Use {@link #setDefaultOption(String, Object)} instead.
     */
    @Deprecated
    public void addDefault(String name, V object) {
        setDefaultOption(name, object);
    }

    /**
     * Returns the selected option. If there is none selected, it will return the
     * default. If there is none selected and no default, then it will return
     * {@code null}.
     *
     * @return the option selected
     */
    public V getSelected() {
        m_mutex.lock();
        try {
            if (m_selected != null) {
                return m_map.get(m_selected);
            } else {
                return m_map.get(m_defaultChoice);
            }
        } finally {
            m_mutex.unlock();
        }
    }

    private String m_selected;
    private final List<NetworkTableEntry> m_activeEntries = new ArrayList<>();
    private final ReentrantLock m_mutex = new ReentrantLock();

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(ACTIVE, () -> {
            m_mutex.lock();
            try {
                if (m_selected != null) {
                    return m_selected;
                } else {
                    return m_defaultChoice;
                }
            } finally {
                m_mutex.unlock();
            }
        }, null);
        m_mutex.lock();
        try {
            m_activeEntries.add(builder.getEntry(ACTIVE));
        } finally {
            m_mutex.unlock();
        }
        builder.addStringProperty(SELECTED, null, val -> {
            m_mutex.lock();
            try {
                m_selected = val;
                for (NetworkTableEntry entry : m_activeEntries) {
                    entry.setString(val);
                }
            } finally {
                m_mutex.unlock();
                hasNewValue = true;
            }
        });
    }
}
