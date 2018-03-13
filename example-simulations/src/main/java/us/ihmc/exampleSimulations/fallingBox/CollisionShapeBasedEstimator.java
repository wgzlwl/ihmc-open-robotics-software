package us.ihmc.exampleSimulations.fallingBox;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   public CollisionShapeBasedEstimator(Robot robot)
   {
      registry = new YoVariableRegistry(robot.getName() + "_registry");
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {

   }
}
