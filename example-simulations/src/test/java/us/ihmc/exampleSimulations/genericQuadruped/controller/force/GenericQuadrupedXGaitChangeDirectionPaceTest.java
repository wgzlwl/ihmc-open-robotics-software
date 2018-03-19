package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitChangeDirectionTest;

public class GenericQuadrupedXGaitChangeDirectionPaceTest extends QuadrupedXGaitChangeDirectionTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 150000)
   public void testPacingForwardThenChangeToLeft()
   {
      super.testPacingForwardThenChangeToLeft();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 150000)
   public void testPacingBackwardThenChangeToLeft()
   {
      super.testPacingBackwardThenChangeToLeft();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 150000)
   public void testPacingForwardThenChangeToRight()
   {
      super.testPacingForwardThenChangeToRight();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 150000)
   public void testPacingBackwardThenChangeToRight()
   {
      super.testPacingBackwardThenChangeToRight();
   }
}