package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public abstract class QuadrupedXGaitTurnTest implements QuadrupedMultiRobotTestInterface
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
   public void testTrottingDiagonallyForwardLeft()
   {

      Vector2D direction = new Vector2D(1.0, 0.3);

      trottingDiagonally(direction, 2, 0.0);
   }

   @Test
   public void testTrottingDiagonallyBackwardLeft()
   {
      Vector2D direction = new Vector2D(-1.0, 0.3);
      trottingDiagonally(direction, 2, 0.0);
   }

   @Test
   public void testTrottingDiagonallyForwardRight()
   {
      Vector2D direction = new Vector2D(1.0, -0.3);
      trottingDiagonally(direction, 2, 0.0);
   }

   @Test
   public void testTrottingDiagonallyBackwardRight()
   {
      Vector2D direction = new Vector2D(-1.0, -0.3);
      trottingDiagonally(direction, 2, 0.0);
   }

   @Test
   public void testPacingDiagonallyForwardLeft()
   {
      Vector2D direction = new Vector2D(1.0, 0.3);
      trottingDiagonally(direction, 2, 180.0);
   }

   @Test
   public void testPacingDiagonallyBackwardLeft()
   {
      Vector2D direction = new Vector2D(-1.0, 0.3);
      trottingDiagonally(direction, 2, 180.0);
   }

   @Test
   public void testPacingDiagonallyForwardRight()
   {
      Vector2D direction = new Vector2D(1.0, -0.3);
      trottingDiagonally(direction, 2, 180.0);
   }

   @Test
   public void testPacingDiagonallyBackwardRight()
   {
      Vector2D direction = new Vector2D(-1.0, -0.3);
      trottingDiagonally(direction, 2, 180.0);
   }

   private void trottingDiagonally(Vector2D direction, double terminalGoalX, double endPhaseShiftInput) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.setYoPlanarVelocityInput(direction);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      if (direction.getX() < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), terminalGoalX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), terminalGoalX));
      }
      conductor.simulate();
   }

   @Test
   public void testTrottingForwardThenTurnLeft()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign);
   }

   @Test
   public void testTrottingBackwardThenTurnLeft()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign);
   }

   @Test
   public void testTrottingForwardThenTurnRight()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign);
   }

   @Test
   public void testTrottingBackwardThenTurnRight()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign);
   }

   @Test
   public void testPacingForwardThenTurnLeft()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign);
   }

   @Test
   public void testPacingBackwardThenTurnLeft()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign);
   }

   @Test
   public void testPacingForwardThenTurnRight()
   {
      List<Vector2D> directions = new ArrayList<>();
      Vector2D firstVelocity = new Vector2D(1.0, 0.0);
      Vector2D secondVelocity = new Vector2D(0.0, -1.0);

      double segmentDuration = 5.0;
      Point2D terminalGoal1 = new Point2D(firstVelocity);
      terminalGoal1.scale(segmentDuration);
      Point2D terminalGoal2 = new Point2D(secondVelocity);
      terminalGoal2.scaleAdd(segmentDuration, terminalGoal1);

      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign);
   }

   @Test
   public void testPacingBackwardThenTurnRight()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign);
   }

   private void gaitThenTurn(List<Vector2D> directions, List<Vector2D> terminalGoals, double endPhaseShiftInput, double turnSign) throws AssertionFailedError
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.setYoPlanarVelocityInput(directions.get(0)); // first direction
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 4.0);

      if (directions.get(0).getX() < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), terminalGoals.get(0).getX()));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), terminalGoals.get(0).getX()));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputX().set(0.0);
      variables.getYoPlanarVelocityInputZ().set(turnSign * 0.4);  // turn
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);

      if (turnSign < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyYaw(), turnSign * 0.5 * Math.PI));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyYaw(), turnSign * 0.5 * Math.PI));
      }
      conductor.simulate();

      variables.getYoPlanarVelocityInputZ().set(0.0);
      variables.setYoPlanarVelocityInput(directions.get(0)); // direction X
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 4.0);

      if (directions.get(1).getY() < 0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyY(), terminalGoals.get(1).getY()));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyY(), terminalGoals.get(1).getY()));
      }
      conductor.simulate();
   }

}
