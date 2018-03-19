package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedXGaitBumpyGroundTrackTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         GroundProfile3D groundProfile;
         groundProfile = createBumpyGroundProfile();
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setGroundProfile3D(groundProfile);
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setUseStateEstimator(false);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting(2);
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testBumpyGroundTrottingForwardTrack()
   {
      gaitBumpyGroundTrack(1, 1, 180.0);
   }

   @Test
   public void testBumpyGroundWalkingForwardTrack()
   {
      gaitBumpyGroundTrack(1, 1, 90.0);
   }

   @Test
   public void testBumpyGroundPacingForwardTrack()
   {
      gaitBumpyGroundTrack(1, 1, 0.0);
   }

   @Test
   public void testBumpyGroundTrottingForwardOtherTrack()
   {
      gaitBumpyGroundTrack(1, -1, 180.0);
   }

   @Test
   public void testBumpyGroundWalkingForwardOtherTrack()
   {
      gaitBumpyGroundTrack(1, -1, 90.0);
   }

   @Test
   public void testBumpyGroundPacingForwardOtherTrack()
   {
      gaitBumpyGroundTrack(1, -1, 0.0);
   }

   @Test
   public void testBumpyGroundTrottingBackwardTrack()
   {
      gaitBumpyGroundTrack(-1, 1, 180.0);
   }

   @Test
   public void testBumpyGroundWalkingBackwardTrack()
   {
      gaitBumpyGroundTrack(-1, 1, 90.0);
   }

   @Test
   public void testBumpyGroundPacingBackwardTrack()
   {
      gaitBumpyGroundTrack(-1, 1, 0.0);
   }

   @Test
   public void testBumpyGroundTrottingBackwardOtherTrack()
   {
      gaitBumpyGroundTrack(-1, -1, 180.0);
   }

   @Test
   public void testBumpyGroundWalkingBackwardOtherTrack()
   {
      gaitBumpyGroundTrack(-1, -1, 90.0);
   }

   @Test
   public void testBumpyGroundPacingBackwardOtherTrack()
   {
      gaitBumpyGroundTrack(-1, -1, 0.0);
   }


   private void gaitBumpyGroundTrack(double directionX, double directionY, double endPhaseShiftInput) throws  AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);


      variables.getYoPlanarVelocityInputX().set(directionX * 1.0);
      variables.getYoPlanarVelocityInputZ().set(directionX*directionY*0.15);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      if(directionX *directionY< 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyYaw(), directionX*directionY *0.25* Math.PI));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyYaw(), directionX*directionY*0.25* Math.PI));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      variables.getYoPlanarVelocityInputZ().set(0.0);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);

      variables.getYoPlanarVelocityInputY().set(directionY*0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      if(directionY< 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyY(), directionY*3));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyY(), directionY*3));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputY().set(0.0);
      variables.getYoPlanarVelocityInputX().set(directionX * -1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);
      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyY(), directionY*2));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyY(), directionY*2));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      variables.getYoPlanarVelocityInputZ().set(directionX*directionY*0.3);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);

      if(directionX *directionY< 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyYaw(), directionX*directionY *0.5* Math.PI));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyYaw(), directionX*directionY*0.5* Math.PI));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputZ().set(0.0);
      variables.getYoPlanarVelocityInputX().set(directionX * 1.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 8.0);

      if(directionX < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyY(), directionY*4));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyY(), directionY*4 ));
      }
      conductor.simulate();



   }

   private static BumpyGroundProfile createBumpyGroundProfile()
   {
      double xAmp1 = 0.05, xFreq1 = 0.2, xAmp2 = 0.01, xFreq2 = 0.2;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.05, yFreq2 = 0.17;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2);
      return groundProfile;
   }

}
