package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableMovableBoxRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;

public class FlatGroundEnvironmentWithStepUpBox implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround;
   private final ArrayList<Robot> environmentRobots = new ArrayList<>();
   private final ContactableRobot box;

   // private final YoFramePoint desiredPosition;
   public FlatGroundEnvironmentWithStepUpBox(YoVariableRegistry registry)
   {

      flatGround = DefaultCommonAvatarEnvironment.setUpGround("Ground");
      ContactableMovableBoxRobot box = ContactableMovableBoxRobot.createContactableWoodBoxRobot("woodBox", 1.0, 1.0, 1.0, 0.1);
      box.setGravity(0.1);
      box.setPosition(0,0,-1);

      environmentRobots.add(box);
      flatGround.addTerrainObject(box);

      this.box = box;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return environmentRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
