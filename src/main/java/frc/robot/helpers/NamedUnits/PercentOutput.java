package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

/**
 * Value guarenteed to be between [-1.0, 1.0]
 */
public class PercentOutput implements Measure<Dimensionless> {
    private final Measure<Dimensionless> measure;

    public PercentOutput(double value) {
        if (value > 1.0) {
            System.err.println(String.format("[WARNING]: Value of %d exceeds maximum of 1.0", value));
            // AnonymousBaseUnit is the only instance of Dimensionless I could find
            // However, instead of being type Dimensionless, it is of type Unit,
            // so we have to cast it back
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(1.0);
        } else if (value < -1.0) {
            System.err.println(String.format("[WARNING]: Value of %d exceeds minimum of -1.0", value));

            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(-1.0);
        } else {
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(value);
        }
    }

    private PercentOutput(Measure<Dimensionless> measure) {
        if (measure.magnitude() > 1.0) {
            System.err.println(String.format("[WARNING]: Measure of %d exceeds maximum of 1.0", measure.magnitude()));
            // AnonymousBaseUnit is the only instance of Dimensionless I could find
            // However, instead of being type Dimensionless, it is of type Unit,
            // so we have to cast it back
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(1.0);
        } else if (measure.magnitude() < -1.0) {
            System.err.println(String.format("[WARNING]: Measure of %d exceeds minimum of -1.0", measure.magnitude()));

            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(-1.0);
        } else {
            this.measure = measure;
        }
    }

    @Override
    public double magnitude() {
        return measure.magnitude();
    }

    @Override
    public double baseUnitMagnitude() {
        return measure.baseUnitMagnitude();
    }

    @Override
    public Dimensionless unit() {
        return measure.unit();
    }

    @Override
    public Measure<Dimensionless> copy() {
        return measure.copy();
    }

    @Override
    public <U2 extends Unit<U2>> Measure<?> times(Measure<U2> other) {
        return measure.times(other);
    }

    @Override
    public PercentOutput times(double multiplier) {
        return new PercentOutput(measure.magnitude() * multiplier);
    }

    @Override
    public PercentOutput divide(Measure<Dimensionless> divisor) {
        return new PercentOutput(measure.divide(divisor));
    }

    @Override
    public PercentOutput divide(double divisor) {
        return new PercentOutput(measure.magnitude() / divisor);
    }

    @Override
    public PercentOutput plus(Measure<Dimensionless> other) {
        return new PercentOutput(measure.plus(other));
    }

    @Override
    public PercentOutput minus(Measure<Dimensionless> other) {
        return new PercentOutput(measure.minus(other));
    }

    @Override
    public PercentOutput negate() {
        return new PercentOutput(measure.negate());
    }

}
