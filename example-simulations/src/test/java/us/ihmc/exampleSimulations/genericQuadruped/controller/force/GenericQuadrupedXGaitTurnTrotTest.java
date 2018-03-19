package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitTurnTest;

public class GenericQuadrupedXGaitTurnTrotTest extends QuadrupedXGaitTurnTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingForwardThenTurnLeft() { super.testTrottingForwardThenTurnLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingBackwardThenTurnLeft() { super.testTrottingBackwardThenTurnLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingForwardThenTurnRight() { super.testTrottingForwardThenTurnRight(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingBackwardThenTurnRight() { super.testTrottingBackwardThenTurnRight(); }



   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyForwardLeft() { super.testTrottingDiagonallyForwardLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyBackwardLeft() { super.testTrottingDiagonallyBackwardLeft(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyForwardRight() { super.testTrottingDiagonallyForwardRight(); }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyBackwardRight() { super.testTrottingDiagonallyBackwardRight(); }


}
