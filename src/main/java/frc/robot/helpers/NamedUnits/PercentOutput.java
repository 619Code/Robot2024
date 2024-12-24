package frc.robot.helpers.NamedUnits;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class PercentOutput implements Measure<Dimensionless> {
    private final Measure<Dimensionless> measure;

    public PercentOutput(double value) {
        if (value > 1.0) {
            // AnonymousBaseUnit is the only instance of Dimensionless I could find
            // However, instead of being type Dimensionless, it is of type Unit,
            // so we have to cast it back
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(1.0);
        } else if (value < -1.0) {
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(-1.0);
        } else {
            this.measure = ((Dimensionless)Units.AnonymousBaseUnit).of(value);
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

}
