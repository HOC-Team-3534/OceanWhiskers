package frc.hocLib.util;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import lombok.Getter;

public class TimeTracker {
    private HashMap<String, Pair<Timer, Counter>> m_map =
            new HashMap<String, Pair<Timer, Counter>>();

    private String prevKey;

    @Getter private boolean enabled;

    @Getter
    private static class Counter {
        private int value;

        void increment() {
            value++;
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) stop();
    }

    public void trackStep(String key) {
        if (enabled) {
            if (prevKey != null) m_map.get(prevKey).getFirst().stop();

            if (!m_map.containsKey(key))
                m_map.put(key, new Pair<Timer, Counter>(new Timer(), new Counter()));

            var step = m_map.get(key);

            step.getFirst().start();
            step.getSecond().increment();

            prevKey = key;
        }
    }

    public void stop() {
        if (prevKey != null) m_map.get(prevKey).getFirst().stop();
        prevKey = null;
    }

    public Time getStepAverageTime(String key) {
        if (!m_map.containsKey(key)) return Seconds.zero();

        var step = m_map.get(key);

        return Seconds.of(step.getFirst().get() / step.getSecond().getValue());
    }

    public String getLongestAverageStep() {
        if (m_map.size() == 0) return "";
        var longestAverageKey = "";
        var longestAverage = Seconds.zero();
        for (var key : m_map.keySet()) {
            var stepAverage = getStepAverageTime(key);
            if (stepAverage.gt(longestAverage)) {
                longestAverageKey = key;
                longestAverage = stepAverage;
            }
        }
        return longestAverageKey;
    }

    public void printStepAverages() {
        for (var key : m_map.keySet()) {
            System.out.println(
                    "Step: "
                            + key
                            + " | Time: "
                            + ((double) Math.round(getStepAverageTime(key).in(Milliseconds) * 1000)
                                    / 1000.0)
                            + " ms");
        }
    }
}
