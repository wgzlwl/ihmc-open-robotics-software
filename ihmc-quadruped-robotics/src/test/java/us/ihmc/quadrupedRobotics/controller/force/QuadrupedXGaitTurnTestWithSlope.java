package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.InclinedGroundProfile;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public abstract class QuadrupedXGaitTurnTestWithSlope implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         InclinedGroundProfile groundProfile = new InclinedGroundProfile(0.1);
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
   public void testTrottingDiagonallyForwardLeftWithSlope()
   {

      Vector2D direction = new Vector2D(1.0, 0.3);

      trottingDiagonally(direction, 2, 0.0, 0.5);
   }

   @Test
   public void testTrottingDiagonallyBackwardLeftWithSlope()
   {
      Vector2D direction = new Vector2D(-1.0, 0.3);
      trottingDiagonally(direction, 2, 0.0, 0.5);
   }

   @Test
   public void testTrottingDiagonallyForwardRightWithSlope()
   {
      Vector2D direction = new Vector2D(1.0, -0.3);
      trottingDiagonally(direction, 2, 0.0, 0.5);
   }

   @Test
   public void testTrottingDiagonallyBackwardRightWithSlope()
   {
      Vector2D direction = new Vector2D(-1.0, -0.3);
      trottingDiagonally(direction, 2, 0.0, 0.5);
   }

   @Test
   public void testPacingDiagonallyForwardLeftWithSlope()
   {
      Vector2D direction = new Vector2D(1.0, 0.3);
      trottingDiagonally(direction, 2, 180.0, 0.5);
   }

   @Test
   public void testPacingDiagonallyBackwardLeftWithSlope()
   {
      Vector2D direction = new Vector2D(-1.0, 0.3);
      trottingDiagonally(direction, 2, 180.0, 0.5);
   }

   @Test
   public void testPacingDiagonallyForwardRightWithSlope()
   {
      Vector2D direction = new Vector2D(1.0, -0.3);
      trottingDiagonally(direction, 2, 180.0, 0.5);
   }

   @Test
   public void testPacingDiagonallyBackwardRightWithSlope()
   {
      Vector2D direction = new Vector2D(-1.0, -0.3);
      trottingDiagonally(direction, 2, 180.0, 0.5);
   }

   private void trottingDiagonally(Vector2D direction, double terminalGoalX, double endPhaseShiftInput, double dropHeightForSlope) throws AssertionFailedError
   {
      variables.getYoComPositionInputZ().set(dropHeightForSlope);

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
   public void testTrottingForwardThenTurnLeftWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign, 0.5);
   }

   @Test
   public void testTrottingBackwardThenTurnLeftWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign, 0.5);
   }

   @Test
   public void testTrottingForwardThenTurnRightWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign, 0.5);
   }

   @Test
   public void testTrottingBackwardThenTurnRightWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 180.0, turnSign, 0.5);
   }

   @Test
   public void testPacingForwardThenTurnLeftWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign, 0.5);
   }

   @Test
   public void testPacingBackwardThenTurnLeftWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, 1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, 1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign, 0.5);
   }

   @Test
   public void testPacingForwardThenTurnRightWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign, 0.5);
   }

   @Test
   public void testPacingBackwardThenTurnRightWithSlope()
   {
      List<Vector2D> directions = new ArrayList<>();
      directions.add(new Vector2D(-1.0, 0.0));
      directions.add(new Vector2D(0.0, -1.0));
      double turnSign = Math.signum(directions.get(0).getX() * directions.get(1).getY());
      List<Vector2D> terminalGoals = new ArrayList<>();
      terminalGoals.add(new Vector2D(-1.0, 0.0));
      terminalGoals.add(new Vector2D(0.0, -1.0));
      gaitThenTurn(directions, terminalGoals, 0.0, turnSign, 0.5);
   }

   private void gaitThenTurn(List<Vector2D> directions, List<Vector2D> terminalGoals, double endPhaseShiftInput, double turnSign, double dropHeightForSlope)
         throws AssertionFailedError
   {
      variables.getYoComPositionInputZ().set(dropHeightForSlope);

      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      variables.getXGaitEndPhaseShiftInput().set(endPhaseShiftInput);
      QuadrupedTestBehaviors.enterXGait(conductor, variables);

      variables.setYoPlanarVelocityInput(directions.get(0));
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
      variables.getYoPlanarVelocityInputZ().set(turnSign * 0.4);
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
      variables.getYoPlanarVelocityInputX().set(directions.get(0).getX());
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
