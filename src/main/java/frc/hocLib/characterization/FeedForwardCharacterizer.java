package frc.hocLib.characterization;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.hocLib.math.PolynomialRegression;
import java.util.LinkedList;
import java.util.List;

public class FeedForwardCharacterizer {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();
    private Time startTime = Milliseconds.of(System.currentTimeMillis());

    public void reset() {
        velocityData.clear();
        voltageData.clear();
    }

    public void start() {
        reset();
        startTime = Milliseconds.of(System.currentTimeMillis());
    }

    public Time getTimeSinceStart() {
        return Milliseconds.of(System.currentTimeMillis()).minus(startTime);
    }

    public void add(LinearVelocity velocity, Voltage voltage) {
        if (Math.abs(velocity.in(MetersPerSecond)) > 1E-4) {
            velocityData.add(Math.abs(velocity.in(MetersPerSecond)));
            voltageData.add(Math.abs(voltage.in(Volts)));
        }
    }

    public void print() {
        double velocityDataArray[] =
                velocityData.stream().mapToDouble(Double::doubleValue).toArray();
        double voltageDataArray[] = voltageData.stream().mapToDouble(Double::doubleValue).toArray();
        double accelerationDataArray[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length - 1; i++) {
            accelerationDataArray[i] = (velocityDataArray[i + 1] - velocityDataArray[i]) / 0.020;
        }
        accelerationDataArray[accelerationDataArray.length - 1] =
                accelerationDataArray[accelerationDataArray.length - 2];
        PolynomialRegression regression =
                new PolynomialRegression(velocityDataArray, voltageDataArray, 1);
        double residualsVoltageVelocityWise[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length; i++) {
            residualsVoltageVelocityWise[i] =
                    voltageDataArray[i] - regression.predict(velocityDataArray[i]);
        }
        PolynomialRegression accelerationRegression =
                new PolynomialRegression(accelerationDataArray, residualsVoltageVelocityWise, 1);
        System.out.println("FF Characterization Results:");
        System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
        System.out.println(
                String.format("\tR2=%.5f", regression.R2(velocityDataArray, voltageDataArray)));
        System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
        System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        System.out.println(String.format("\tkA=%.5f", accelerationRegression.beta(1)));
    }
}
