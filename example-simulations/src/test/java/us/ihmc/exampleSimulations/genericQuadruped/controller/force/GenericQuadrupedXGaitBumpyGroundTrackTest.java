package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitBumpyGroundTrackTest;

public class GenericQuadrupedXGaitBumpyGroundTrackTest extends QuadrupedXGaitBumpyGroundTrackTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundTrottingForwardTrack()
   {
      super.testBumpyGroundTrottingForwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundWalkingForwardTrack()
   {
      super.testBumpyGroundWalkingForwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundPacingForwardTrack()
   {
      super.testBumpyGroundPacingForwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundTrottingForwardOtherTrack()
   {
      super.testBumpyGroundTrottingForwardOtherTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundWalkingForwardOtherTrack()
   {
      super.testBumpyGroundWalkingForwardOtherTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundPacingForwardOtherTrack()
   {
      super.testBumpyGroundPacingForwardOtherTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundTrottingBackwardTrack()
   {
      super.testBumpyGroundTrottingBackwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundWalkingBackwardTrack()
   {
      super.testBumpyGroundWalkingBackwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundPacingBackwardTrack()
   {
      super.testBumpyGroundPacingBackwardTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundTrottingBackwardOtherTrack()
   {
      super.testBumpyGroundTrottingBackwardOtherTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundWalkingBackwardOtherTrack()
   {
      super.testBumpyGroundWalkingBackwardOtherTrack();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 300000)
   public void testBumpyGroundPacingBackwardOtherTrack()
   {
      super.testBumpyGroundPacingBackwardOtherTrack();
   }

}