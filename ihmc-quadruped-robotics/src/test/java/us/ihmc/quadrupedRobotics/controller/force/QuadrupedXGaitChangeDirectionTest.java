package us.ihmc.quadrupedRobotics.controller.force;

import gnu.trove.list.array.TDoubleArrayList;
import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

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
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 180.0);
   }

   @Test
   public void testTrottingBackwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 180.0);
   }

   @Test
   public void testTrottingForwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 180.0);
   }

   @Test
   public void testTrottingBackwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 180.0);
   }



   @Test
   public void testPacingForwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 0.0);
   }

   @Test
   public void testPacingBackwardThenChangeToLeft()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, 0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 0.0);
   }

   @Test
   public void testPacingForwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 0.0);
   }

   @Test
   public void testPacingBackwardThenChangeToRight()
   {
      List<Vector2D> velocities = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(-1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -0.4);
      velocities.add(firstVelocity);
      velocities.add(secondVelocity);
      double segmentDuration = 2.0;
      List<Point2D> terminalGoals = new ArrayList<>();
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);
      terminalGoals.add(terminalGoal1);
      terminalGoals.add(terminalGoal2);
      gaitThenChangeDirection(velocities, terminalGoals, 0.0);
   }


   private void gaitThenChangeDirection(List<Vector2D> velocities, List<Point2D> terminalGoals, double endPhaseShiftInput) throws  AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);


      variables.setYoPlanarVelocityInput(velocities.get(0));
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);

      Vector2D plusMinusVector = new Vector2D(0.05, 0.05);
      BoundingBox2D boundingBox = BoundingBox2D.createUsingCenterAndPlusMinusVector(terminalGoals.get(0), plusMinusVector);
      conductor.addTerminalGoal(YoVariableTestGoal.pointIn2DBoundingBox(variables.getRobotBodyX(), variables.getRobotBodyY(), boundingBox));
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      variables.setYoPlanarVelocityInput(velocities.get(1));
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);

      Vector2D plusMinusVector1 = new Vector2D(0.4, 0.4);
      BoundingBox2D boundingBox1 = BoundingBox2D.createUsingCenterAndPlusMinusVector(terminalGoals.get(1), plusMinusVector1);
      conductor.addTerminalGoal(YoVariableTestGoal.pointIn2DBoundingBox(variables.getRobotBodyX(), variables.getRobotBodyY(), boundingBox1));
      conductor.simulate();
   }


}
