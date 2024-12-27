package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

public class Degrees implements Measure<Angle>{
    private final Measure<Angle> measure;

    public Degrees(double magnitude) {
        measure = Units.Degrees.of(magnitude);
    }

    // Prevent an extra allocation
    private Degrees(Measure<Angle> measure) {
        this.measure = measure;
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
    public Angle unit() {
        return measure.unit();
    }

    @Override
    public Measure<Angle> copy() {
        return measure.copy();
    }

    @Override
    public Degrees times(double multiplier) {
        return new Degrees(measure.times(multiplier));
    }

    @Override
    public <U2 extends Unit<U2>> Measure<?> times(Measure<U2> other) {
        return measure.times(other);
    }

    @Override
    public Degrees divide(Measure<Dimensionless> divisor) {
        return new Degrees(measure.divide(divisor));
    }

    @Override
    public Degrees divide(double divisor) {
        return new Degrees(measure.divide(divisor));
    }

    @Override
    public Degrees plus(Measure<Angle> other) {
        return new Degrees(measure.plus(other));
    }

    @Override
    public Degrees minus(Measure<Angle> other) {
        return new Degrees(measure.minus(other));
    }

    @Override
    public Degrees negate() {
        return new Degrees(measure.negate());
    }

}
