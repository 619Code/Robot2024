package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class RevolutionsPerMinute implements Measure<Velocity<Angle>>{
    // TODO: To save allocations/speed, maybe make this a MutableMeasure?
    private Measure<Velocity<Angle>> measure;

    public RevolutionsPerMinute(double value) {
        this.measure = Units.RPM.of(value);
    }

    // Prevent an extra allocation
    private RevolutionsPerMinute(Measure<Velocity<Angle>> measure) {
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
    public Velocity<Angle> unit() {
        return measure.unit();
    }

    @Override
    public Measure<Velocity<Angle>> copy() {
        return measure.copy();
    }

    @Override
    public RevolutionsPerMinute times(double multiplier) {
        return new RevolutionsPerMinute(measure.times(multiplier));
    }

    @Override
    public <U2 extends Unit<U2>> Measure<?> times(Measure<U2> other) {
        return measure.times(other);
    }

    @Override
    public RevolutionsPerMinute divide(Measure<Dimensionless> divisor) {
        return new RevolutionsPerMinute(measure.divide(divisor));
    }

    @Override
    public RevolutionsPerMinute divide(double divisor) {
        return new RevolutionsPerMinute(measure.divide(divisor));
    }

    @Override
    public RevolutionsPerMinute plus(Measure<Velocity<Angle>> other) {
        return new RevolutionsPerMinute(measure.plus(other));
    }

    @Override
    public RevolutionsPerMinute minus(Measure<Velocity<Angle>> other) {
        return new RevolutionsPerMinute(measure.minus(other));
    }

    @Override
    public RevolutionsPerMinute negate() {
        return new RevolutionsPerMinute(measure.negate());
    }
}
