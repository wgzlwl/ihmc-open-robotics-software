package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitTurnTestWithSlope;

import java.io.IOException;

public class GenericQuadrupedXGaitTurnTestWithSlope extends QuadrupedXGaitTurnTestWithSlope
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingForwardThenTurnLeftWithSlope()
   {
      super.testTrottingForwardThenTurnLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingBackwardThenTurnLeftWithSlope()
   {
      super.testTrottingBackwardThenTurnLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingForwardThenTurnRightWithSlope()
   {
      super.testTrottingForwardThenTurnRightWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingBackwardThenTurnRightWithSlope()
   {
      super.testTrottingBackwardThenTurnRightWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyForwardLeftWithSlope()
   {
      super.testTrottingDiagonallyForwardLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyBackwardLeftWithSlope()
   {
      super.testTrottingDiagonallyBackwardLeftWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyForwardRightWithSlope()
   {
      super.testTrottingDiagonallyForwardRightWithSlope();
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 200000)
   public void testTrottingDiagonallyBackwardRightWithSlope()
   {
      super.testTrottingDiagonallyBackwardRightWithSlope();
   }
}