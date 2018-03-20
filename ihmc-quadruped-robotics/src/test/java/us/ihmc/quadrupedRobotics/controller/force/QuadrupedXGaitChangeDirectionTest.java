package us.ihmc.quadrupedRobotics.controller.force;

import gnu.trove.list.array.TDoubleArrayList;
import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public abstract class QuadrupedXGaitChangeDirectionTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
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
   public void testTrottingForwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 180.0);
   }

   @Test
   public void testTrottingBackwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 180.0);
   }

   @Test
   public void testTrottingForwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 180.0);
   }

   @Test
   public void testTrottingBackwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 180.0);
   }



   @Test
   public void testPacingForwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 0.0);
   }

   @Test
   public void testPacingBackwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 0.0);
   }

   @Test
   public void testPacingForwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 0.0);
   }

   @Test
   public void testPacingBackwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double[] segmentDurations = new double[]{2.0,2.0};
      gaitThenChangeDirection(velocities, segmentDurations, 0.0);
   }


   private void gaitThenChangeDirection(List<Vector2D> velocities, double[] segmentDurations , double endPhaseShiftInput) throws  AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);


      variables.setYoPlanarVelocityInput(velocities.get(0));
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);

      Point2D terminalGoal = new Point2D(velocities.get(0));
      terminalGoal.scale(segmentDurations[0]);
      RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      transformGenerator.rotate(0.5*Math.PI, Axis.Z);
      velocities.get(0).applyTransform(transformGenerator.getRigidBodyTransformCopy());
      Line2D perpendicularLine = new Line2D(terminalGoal,velocities.get(0));
      conductor.addTerminalGoal(YoVariableTestGoal.nearLine(variables.getRobotBodyX(),variables.getRobotBodyY(), perpendicularLine, 0.05));
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      Point2D newStartPoint = new Point2D(variables.getRobotBodyX().getDoubleValue(), variables.getRobotBodyY().getDoubleValue());
      variables.setYoPlanarVelocityInput(velocities.get(1));
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);


      Point2D terminalGoal1 = new Point2D(velocities.get(1));
      terminalGoal1.scaleAdd(segmentDurations[1], newStartPoint);
      velocities.get(1).applyTransform(transformGenerator.getRigidBodyTransformCopy());
      Line2D perpendicularLine1 = new Line2D(terminalGoal1, velocities.get(1));
      conductor.addTerminalGoal(YoVariableTestGoal.nearLine(variables.getRobotBodyX(),variables.getRobotBodyY(), perpendicularLine1, 0.05));
      conductor.simulate();
   }


}
