package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;

public class Radians implements Measure<Angle>{
    private final Measure<Angle> measure;

    public Radians(double magnitude) {
        measure = Units.Radians.of(magnitude);
    }

    // Prevent an extra allocation
    private Radians(Measure<Angle> measure) {
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
    public Radians times(double multiplier) {
        return new Radians(measure.times(multiplier));
    }

    @Override
    public <U2 extends Unit<U2>> Measure<?> times(Measure<U2> other) {
        return measure.times(other);
    }

    @Override
    public Radians divide(Measure<Dimensionless> divisor) {
        return new Radians(measure.divide(divisor));
    }

    @Override
    public Radians divide(double divisor) {
        return new Radians(measure.divide(divisor));
    }

    @Override
    public Radians plus(Measure<Angle> other) {
        return new Radians(measure.plus(other));
    }

    @Override
    public Radians minus(Measure<Angle> other) {
        return new Radians(measure.minus(other));
    }

    @Override
    public Radians negate() {
        return new Radians(measure.negate());
    }

}
