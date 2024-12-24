package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class RevolutionsPerMinute implements Measure<Velocity<Angle>>{
    private Measure<Velocity<Angle>> measure;

    public RevolutionsPerMinute(double value) {
        this.measure = Units.RPM.of(value);
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
}
