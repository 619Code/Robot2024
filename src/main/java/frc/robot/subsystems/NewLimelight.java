package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.Crashboard;

public class NewLimelight extends SubsystemBase{
    
    NetworkTableEntry tx;
   // NetworkTableEntry ty;

    public NewLimelight(){
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
   //     ty = table.getEntry("ty");

    }

    public void SendRandomDataToShuffleboard(){
        
        //read values periodically
        double x = tx.getDouble(0.0);
     //   double y = ty.getDouble(0.0);

        //post to smart dashboard periodically
        Crashboard.toDashboard("X: ", x, "Limelight");
  //      Crashboard.toDashboard("Y: ", y, "Limelight");

    }

    public double GetGoofyAhhHeading(){
        return tx.getDouble(0.0);
    }
    
    // DoubleArraySubscriber botPoseSub;

    // public NewLimelight(){

    //     NetworkTable limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //     botPoseSub = limeLightTable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{});

    // }

    // public void GetLocalPosition(){

    //     double[] data = botPoseSub.get();

    //     if(data.length < 1) return;

    //     Crashboard.toDashboard("Local X: ", data[0], "Limelight");

    // }
}
