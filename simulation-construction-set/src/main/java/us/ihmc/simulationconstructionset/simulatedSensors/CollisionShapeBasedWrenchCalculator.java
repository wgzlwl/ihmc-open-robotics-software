package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final String forceSensorName;
   private final List<ExternalForcePoint> contactPoints;
   private final Joint forceTorqueSensorJoint;

   private final RigidBodyTransform transformToParentJoint;

   private boolean doWrenchCorruption = false;
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F corruptionMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

   public CollisionShapeBasedWrenchCalculator(String forceSensorName, List<ExternalForcePoint> contactPoints, Joint forceTorqueSensorJoint,
                                              RigidBodyTransform transformToParentJoint, YoVariableRegistry registry)
   {
      this.forceSensorName = forceSensorName;
      this.contactPoints = contactPoints;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
      this.transformToParentJoint = new RigidBodyTransform(transformToParentJoint);
   }

   public void initializeExternalForcePoints(List<ExternalForcePoint> contactPoints)
   {
      this.contactPoints.clear();
      this.contactPoints.addAll(contactPoints);
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToParentJoint);
   }

   @Override
   public void calculate()
   {
      wrenchMatrix.zero();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         ExternalForcePoint contactPoint = contactPoints.get(i);
         Vector3D force = new Vector3D();
         Vector3D position = new Vector3D();

         contactPoint.getForce(force);
         contactPoint.getPosition(position);

         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.getX());
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.getY());
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.getZ());
      }

      if (doWrenchCorruption)
      {
         for (int i = 0; i < SpatialForceVector.SIZE; i++)
         {
            wrenchMatrix.add(i, 0, corruptionMatrix.get(i, 0));
         }
      }
   }

   @Override
   public DenseMatrix64F getWrench()
   {
      return wrenchMatrix;
   }

   @Override
   public Joint getJoint()
   {
      return forceTorqueSensorJoint;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      this.corruptionMatrix.add(row, 0, value);
   }

   @Override
   public void setDoWrenchCorruption(boolean value)
   {
      doWrenchCorruption = value;
   }

}
