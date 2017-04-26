package us.ihmc.commonWalkingControlModules.controlModules.kneeAngle;

import us.ihmc.commonWalkingControlModules.configurations.StraightLegWalkingParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

import java.util.ArrayList;
import java.util.List;

public class KneeControlModule
{
   public enum KneeControlType
   {
      STRAIGHTEN_TO_STRAIGHT, STRAIGHT, STRAIGHTEN_TO_CONTROLLABLE, BENT, CONTROLLABLE
   }

   private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;
   private static final boolean SCALE_STRAIGHT_GAIN_WITH_ERROR = false;

   private final YoVariableRegistry registry;

   private final EnumYoVariable<KneeControlType> requestedState;
   private final GenericStateMachine<KneeControlType, AbstractKneeControlState> stateMachine;

   private final BooleanYoVariable includeHipPitchPrivilegedConfiguration;

   private final DoubleYoVariable hipPrivilegedWeight;
   private final DoubleYoVariable hipPrivilegedPositionGain;
   private final DoubleYoVariable hipPrivilegedVelocityGain;

   private final DoubleYoVariable kneeStraightPrivilegedWeight;
   private final DoubleYoVariable kneeStraightPrivilegedPositionGain;
   private final DoubleYoVariable kneeStraightPrivilegedVelocityGain;

   private final DoubleYoVariable kneeBentPrivilegedWeight;
   private final DoubleYoVariable kneeBentPrivilegedPositionGain;
   private final DoubleYoVariable kneeBentPrivilegedVelocityGain;

   private final DoubleYoVariable privilegedMaxAcceleration;

   private final DoubleYoVariable desiredAngle;
   private final DoubleYoVariable desiredAngleWhenStraight;

   private final DoubleYoVariable straighteningSpeed;

   private final BooleanYoVariable activelyControl;

   private final YoPDGains jointspaceGains;
   private final DoubleYoVariable jointspaceWeight;

   public KneeControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, StraightLegWalkingParameters straightLegWalkingParameters,
         YoVariableRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Knee";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

      includeHipPitchPrivilegedConfiguration = new BooleanYoVariable(namePrefix + "IncludeHipPitchPrivilegedConfiguration", registry);
      includeHipPitchPrivilegedConfiguration.set(straightLegWalkingParameters.includeHipPitchPrivilegedConfiguration());

      activelyControl = new BooleanYoVariable(namePrefix + "ActivelyControl", registry);
      activelyControl.set(false);

      jointspaceWeight = new DoubleYoVariable(namePrefix + "JointspaceWeight", registry);
      jointspaceWeight.set(1.0);

      jointspaceGains = new YoPDGains(namePrefix, registry);
      jointspaceGains.setKp(40.0);
      jointspaceGains.setKd(6.0);

      hipPrivilegedWeight = new DoubleYoVariable(sidePrefix + "HipPrivilegedWeight", registry);
      hipPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "HipPrivilegedKp", registry);
      hipPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "HipPrivilegedKv", registry);

      kneeStraightPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedWeight", registry);
      kneeStraightPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKp", registry);
      kneeStraightPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeStraightPrivilegedKv", registry);

      kneeBentPrivilegedWeight = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedWeight", registry);
      kneeBentPrivilegedPositionGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKp", registry);
      kneeBentPrivilegedVelocityGain = new DoubleYoVariable(sidePrefix + "KneeBentPrivilegedKv", registry);

      privilegedMaxAcceleration = new DoubleYoVariable(namePrefix + "PrivilegedMaxAcceleration", registry);

      hipPrivilegedWeight.set(straightLegWalkingParameters.getHipPrivilegedWeight());
      hipPrivilegedPositionGain.set(straightLegWalkingParameters.getHipPrivilegedConfigurationGain());
      hipPrivilegedVelocityGain.set(straightLegWalkingParameters.getHipPrivilegedVelocityGain());

      kneeStraightPrivilegedWeight.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedWeight());
      kneeStraightPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedConfigurationGain());
      kneeStraightPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeStraightLegPrivilegedVelocityGain());

      kneeBentPrivilegedWeight.set(straightLegWalkingParameters.getKneeBentLegPrivilegedWeight());
      kneeBentPrivilegedPositionGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedConfigurationGain());
      kneeBentPrivilegedVelocityGain.set(straightLegWalkingParameters.getKneeBentLegPrivilegedVelocityGain());

      privilegedMaxAcceleration.set(straightLegWalkingParameters.getPrivilegedMaxAcceleration());

      desiredAngle = new DoubleYoVariable(namePrefix + "DesiredAngle", registry);
      desiredAngle.set(straightLegWalkingParameters.getStraightKneeAngle());

      desiredAngleWhenStraight = new DoubleYoVariable(namePrefix + "DesiredAngleWhenStraight", registry);
      desiredAngleWhenStraight.set(straightLegWalkingParameters.getStraightKneeAngle());

      straighteningSpeed = new DoubleYoVariable(namePrefix + "StraighteningSpeed", registry);
      straighteningSpeed.set(straightLegWalkingParameters.getSpeedForStanceLegStraightening());

      // set up states and state machine
      DoubleYoVariable time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", KneeControlType.class, time, registry);
      requestedState = EnumYoVariable.create(namePrefix + "RequestedState", "", KneeControlType.class, registry, true);
      requestedState.set(null);

      OneDoFJoint hipJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJoint kneeJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);

      List<AbstractKneeControlState> states = new ArrayList<>();

      AbstractKneeControlState straighteningToStraightState = new StraightenToStraightControlState(hipJoint, kneeJoint, straighteningSpeed);
      states.add(straighteningToStraightState);
      AbstractKneeControlState straightState = new StraightKneeControlState(hipJoint, kneeJoint);
      states.add(straightState);
      AbstractKneeControlState bentState = new BentKneeControlState(hipJoint, kneeJoint);
      states.add(bentState);
      AbstractKneeControlState straighteningToControlState = new StraightenToControllableControlState(hipJoint, kneeJoint, straighteningSpeed);
      states.add(straighteningToControlState);
      AbstractKneeControlState controlledState = new ControllableKneeControlState(hipJoint, kneeJoint);
      states.add(controlledState);

      straighteningToStraightState.setDefaultNextState(KneeControlType.STRAIGHT);
      straighteningToControlState.setDefaultNextState(KneeControlType.CONTROLLABLE);

      setupStateMachine(states, straightLegWalkingParameters.attemptToStraightenLegs());

      parentRegistry.addChild(registry);
   }

   private void setupStateMachine(List<AbstractKneeControlState> states, boolean attemptToStraightenLegs)
   {
      for (AbstractKneeControlState fromState : states)
      {
         for (AbstractKneeControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
      }

      for (AbstractKneeControlState state : states)
      {
         stateMachine.addState(state);
      }

      if (attemptToStraightenLegs)
         stateMachine.setCurrentState(KneeControlType.STRAIGHT);
      else
         stateMachine.setCurrentState(KneeControlType.BENT);

   }

   public void initialize()
   {
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.getCurrentState().doAction();
   }

   public void setKneeAngleState(KneeControlType controlType)
   {
      requestedState.set(controlType);
   }

   public KneeControlType getCurrentKneeControlState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getJointspaceFeedbackControlCommand();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return stateMachine.getCurrentState().getPrivilegedConfigurationCommand();
   }

   private abstract class AbstractKneeControlState extends FinishableState<KneeControlType>
   {
      protected final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

      public AbstractKneeControlState(KneeControlType stateEnum)
      {
         super(stateEnum);
      }

      public JointspaceFeedbackControlCommand getJointspaceFeedbackControlCommand()
      {
         return null;
      }

      public PrivilegedConfigurationCommand getPrivilegedConfigurationCommand()
      {
         return privilegedConfigurationCommand;
      }
   }

   private class StraightenToStraightControlState extends StraighteningKneeControlState
   {
      public StraightenToStraightControlState(OneDoFJoint hipJoint, OneDoFJoint kneeJoint, DoubleYoVariable straighteningSpeed)
      {
         super(KneeControlType.STRAIGHTEN_TO_STRAIGHT, hipJoint, kneeJoint, straighteningSpeed);
      }
   }

   private class StraightenToControllableControlState extends StraighteningKneeControlState
   {
      public StraightenToControllableControlState(OneDoFJoint hipJoint, OneDoFJoint kneeJoint, DoubleYoVariable straighteningSpeed)
      {
         super(KneeControlType.STRAIGHTEN_TO_CONTROLLABLE, hipJoint, kneeJoint, straighteningSpeed);
      }
   }

   private class StraighteningKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint hipJoint;
      private final OneDoFJoint kneeJoint;

      private final DoubleYoVariable yoStraighteningSpeed;

      private double startingPosition;

      private double timeUntilStraight;
      private double straighteningSpeed;

      private double dwellTime;
      private double desiredPrivilegedPosition;

      private double previousTime;

      public StraighteningKneeControlState(KneeControlType stateEnum, OneDoFJoint hipJoint, OneDoFJoint kneeJoint, DoubleYoVariable straighteningSpeed)
      {
         super(stateEnum);

         this.hipJoint = hipJoint;
         this.kneeJoint = kneeJoint;

         this.yoStraighteningSpeed = straighteningSpeed;
      }

      @Override
      public boolean isDone()
      {
         return getTimeInCurrentState() > (timeUntilStraight + dwellTime);
      }

      @Override
      public void doAction()
      {
         double estimatedDT = estimateDT();
         double currentPosition = kneeJoint.getQ();

         if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
         {
            if (currentPosition > startingPosition) // the knee is bending
               dwellTime += estimatedDT;
            else
               desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }
         else
         {
            desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
         }

         double gainModifier = 1.0;
         if (SCALE_STRAIGHT_GAIN_WITH_ERROR)
         {
            double absoluteError = Math.abs(kneeJoint.getQ() - desiredAngleWhenStraight.getDoubleValue()) / (2.0 * Math.PI);
            gainModifier = 1.0 / (1.0 + absoluteError);
         }

         privilegedConfigurationCommand.clear();

         int kneeJointIndex = 0;
         if (includeHipPitchPrivilegedConfiguration.getBooleanValue())
         {
            int hipJointIndex = 0;
            privilegedConfigurationCommand.addJoint(hipJoint, PrivilegedConfigurationOption.AT_ZERO);
            privilegedConfigurationCommand.setWeight(hipJointIndex, hipPrivilegedWeight.getDoubleValue());
            privilegedConfigurationCommand.setConfigurationGain(hipJointIndex, gainModifier * hipPrivilegedPositionGain.getDoubleValue());
            privilegedConfigurationCommand.setVelocityGain(hipJointIndex, hipPrivilegedVelocityGain.getDoubleValue());
            privilegedConfigurationCommand.setMaxAcceleration(hipJointIndex, privilegedMaxAcceleration.getDoubleValue());

            kneeJointIndex++;
         }

         privilegedConfigurationCommand.addJoint(kneeJoint, desiredPrivilegedPosition);
         privilegedConfigurationCommand.setWeight(kneeJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneeJointIndex, gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneeJointIndex, kneeStraightPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneeJointIndex, privilegedMaxAcceleration.getDoubleValue());

         if (isDone())
            transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         startingPosition = kneeJoint.getQ();

         straighteningSpeed = yoStraighteningSpeed.getDoubleValue();
         timeUntilStraight = (startingPosition - desiredAngleWhenStraight.getDoubleValue()) / straighteningSpeed;
         timeUntilStraight = Math.max(timeUntilStraight, 0.0);

         desiredPrivilegedPosition = startingPosition;

         previousTime = 0.0;
         dwellTime = 0.0;
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      private double estimateDT()
      {
         double currentTime = getTimeInCurrentState();

         double estimatedDT = currentTime - previousTime;
         previousTime = currentTime;

         return estimatedDT;
      }
   }

   private class StraightKneeControlState extends AbstractKneeControlState
   {
      private final JointspaceFeedbackControlCommand jointspaceFeedbackControlCommand = new JointspaceFeedbackControlCommand();

      private final OneDoFJoint hipJoint;
      private final OneDoFJoint kneeJoint;

      public StraightKneeControlState(OneDoFJoint hipJoint, OneDoFJoint kneeJoint)
      {
         super(KneeControlType.STRAIGHT);

         this.hipJoint = hipJoint;
         this.kneeJoint = kneeJoint;


         jointspaceFeedbackControlCommand.addJoint(kneeJoint, Double.NaN, Double.NaN, Double.NaN);
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         double gainModifier = 1.0;
         if (SCALE_STRAIGHT_GAIN_WITH_ERROR)
         {
            double absoluteError = Math.abs(kneeJoint.getQ() - desiredAngleWhenStraight.getDoubleValue()) / (2.0 * Math.PI);
            gainModifier = 1.0 / (1.0 + absoluteError);
         }

         privilegedConfigurationCommand.clear();

         int kneeJointIndex = 0;
         if (includeHipPitchPrivilegedConfiguration.getBooleanValue())
         {
            int hipJointIndex = 0;
            privilegedConfigurationCommand.addJoint(hipJoint, PrivilegedConfigurationOption.AT_ZERO);
            privilegedConfigurationCommand.setWeight(hipJointIndex, hipPrivilegedWeight.getDoubleValue());
            privilegedConfigurationCommand.setConfigurationGain(hipJointIndex, gainModifier * hipPrivilegedPositionGain.getDoubleValue());
            privilegedConfigurationCommand.setVelocityGain(hipJointIndex, hipPrivilegedVelocityGain.getDoubleValue());
            privilegedConfigurationCommand.setMaxAcceleration(hipJointIndex, privilegedMaxAcceleration.getDoubleValue());

            kneeJointIndex++;
         }

         privilegedConfigurationCommand.addJoint(kneeJoint, desiredAngleWhenStraight.getDoubleValue());
         privilegedConfigurationCommand.setWeight(kneeJointIndex, kneeStraightPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneeJointIndex, gainModifier * kneeStraightPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneeJointIndex, kneeStraightPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneeJointIndex, privilegedMaxAcceleration.getDoubleValue());

         jointspaceFeedbackControlCommand.setOneDoFJoint(0, desiredAngleWhenStraight.getDoubleValue(), 0.0, 0.0);
         jointspaceFeedbackControlCommand.setGains(jointspaceGains);
         jointspaceFeedbackControlCommand.setWeightForSolver(jointspaceWeight.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      @Override
      public JointspaceFeedbackControlCommand getJointspaceFeedbackControlCommand()
      {
         if (activelyControl.getBooleanValue())
            return jointspaceFeedbackControlCommand;
         else
            return null;
      }
   }

   private class BentKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint hipJoint;
      private final OneDoFJoint kneeJoint;

      public BentKneeControlState(OneDoFJoint hipJoint, OneDoFJoint kneeJoint)
      {
         super(KneeControlType.BENT);

         this.hipJoint = hipJoint;
         this.kneeJoint = kneeJoint;
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.clear();

         int kneeJointIndex = 0;
         if (includeHipPitchPrivilegedConfiguration.getBooleanValue())
         {
            int hipJointIndex = 0;
            privilegedConfigurationCommand.addJoint(hipJoint, PrivilegedConfigurationOption.AT_ZERO);
            privilegedConfigurationCommand.setWeight(hipJointIndex, hipPrivilegedWeight.getDoubleValue());
            privilegedConfigurationCommand.setConfigurationGain(hipJointIndex, hipPrivilegedPositionGain.getDoubleValue());
            privilegedConfigurationCommand.setVelocityGain(hipJointIndex, hipPrivilegedVelocityGain.getDoubleValue());
            privilegedConfigurationCommand.setMaxAcceleration(hipJointIndex, privilegedMaxAcceleration.getDoubleValue());

            kneeJointIndex++;
         }

         privilegedConfigurationCommand.addJoint(kneeJoint, PrivilegedConfigurationOption.AT_MID_RANGE);
         privilegedConfigurationCommand.setWeight(kneeJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneeJointIndex, kneeBentPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneeJointIndex, kneeBentPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneeJointIndex, privilegedMaxAcceleration.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

   private class ControllableKneeControlState extends AbstractKneeControlState
   {
      private final OneDoFJoint hipJoint;
      private final OneDoFJoint kneeJoint;

      public ControllableKneeControlState(OneDoFJoint hipJoint, OneDoFJoint kneeJoint)
      {
         super(KneeControlType.CONTROLLABLE);

         this.hipJoint = hipJoint;
         this.kneeJoint = kneeJoint;
      }

      @Override
      public boolean isDone()
      {
         return false;
      }

      @Override
      public void doAction()
      {
         privilegedConfigurationCommand.clear();

         int kneeJointIndex = 0;
         if (includeHipPitchPrivilegedConfiguration.getBooleanValue())
         {
            int hipJointIndex = 0;
            privilegedConfigurationCommand.addJoint(hipJoint, PrivilegedConfigurationOption.AT_ZERO);
            privilegedConfigurationCommand.setWeight(hipJointIndex, hipPrivilegedWeight.getDoubleValue());
            privilegedConfigurationCommand.setConfigurationGain(hipJointIndex, hipPrivilegedPositionGain.getDoubleValue());
            privilegedConfigurationCommand.setVelocityGain(hipJointIndex, hipPrivilegedVelocityGain.getDoubleValue());
            privilegedConfigurationCommand.setMaxAcceleration(hipJointIndex, privilegedMaxAcceleration.getDoubleValue());

            kneeJointIndex++;
         }

         privilegedConfigurationCommand.addJoint(kneeJoint, desiredAngle.getDoubleValue());
         privilegedConfigurationCommand.setWeight(kneeJointIndex, kneeBentPrivilegedWeight.getDoubleValue());
         privilegedConfigurationCommand.setConfigurationGain(kneeJointIndex, kneeBentPrivilegedPositionGain.getDoubleValue());
         privilegedConfigurationCommand.setVelocityGain(kneeJointIndex, kneeBentPrivilegedVelocityGain.getDoubleValue());
         privilegedConfigurationCommand.setMaxAcceleration(kneeJointIndex, privilegedMaxAcceleration.getDoubleValue());
      }

      @Override
      public void doTransitionIntoAction()
      {
         desiredAngle.set(desiredAngleWhenStraight.getDoubleValue());
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }
}