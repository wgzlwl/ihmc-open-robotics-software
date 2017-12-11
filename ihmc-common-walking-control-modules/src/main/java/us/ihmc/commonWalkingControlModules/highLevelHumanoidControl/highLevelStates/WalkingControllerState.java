package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class WalkingControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.WALKING;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final WholeBodyControllerCore controllerCore;
   private final WalkingHighLevelHumanoidController walkingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private boolean setupInverseDynamicsSolver = true;
   private boolean setupInverseKinematicsSolver = false;
   private boolean setupVirtualModelControlSolver = false;

   private final JointDesiredOutputList additionalJointLevelSettings;

   private final AccelerationIntegrationParameterHelper accelerationIntegrationParameterHelper;

   public WalkingControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState);

      // create walking controller
      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);

      // create controller core
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      InverseDynamicsJoint[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controllerToolbox.getControlDT(), controllerToolbox.getGravityZ(), rootJoint,
                                                                            jointsToOptimizeFor, centerOfMassFrame,
                                                                            walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                            controllerToolbox.getYoGraphicsListRegistry(), registry);
      toolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      if (setupInverseDynamicsSolver)
         toolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      if (setupInverseKinematicsSolver)
         toolbox.setupForInverseKinematicsSolver();
      if (setupVirtualModelControlSolver)
      {
         RigidBody[] controlledBodies = {fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(RobotSide.RIGHT)};
         toolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controlledBodies, controllerToolbox.getContactablePlaneBodies());
      }
      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controllerToolbox.getFullRobotModel()
                                                                                                                                        .getOneDoFJoints());
      controllerCore = new WholeBodyControllerCore(toolbox, template, lowLevelControllerOutput, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();
      walkingController.setControllerCoreOutput(controllerCoreOutput);

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      additionalJointLevelSettings = new JointDesiredOutputList(controlledJoints);

      List<String> positionControlledJoints = new ArrayList<>();
      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         JointDesiredControlMode jointDesiredControlMode = highLevelControllerParameters.getJointDesiredControlMode(controlledJoint.getName(), controllerState);
         double desiredJointStiffness = highLevelControllerParameters.getDesiredJointStiffness(controlledJoint.getName(), controllerState);
         double desiredJointDamping = highLevelControllerParameters.getDesiredJointDamping(controlledJoint.getName(), controllerState);
         double velocityScaling = highLevelControllerParameters.getJointVelocityScaling(controlledJoint.getName(), controllerState);

         // Integration acceleration must be activated for these joints.
         if (jointDesiredControlMode == JointDesiredControlMode.POSITION)
         {
            positionControlledJoints.add(controlledJoint.getName());
         }

         JointDesiredOutput jointDesiredOutput = additionalJointLevelSettings.getJointDesiredOutput(controlledJoint);
         jointDesiredOutput.setControlMode(jointDesiredControlMode);
         jointDesiredOutput.setStiffness(desiredJointStiffness);
         jointDesiredOutput.setDamping(desiredJointDamping);
         jointDesiredOutput.setVelocityScaling(velocityScaling);
      }

      OneDoFJoint[] controlledOneDofJoints = ScrewTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class);
      accelerationIntegrationParameterHelper = new AccelerationIntegrationParameterHelper(highLevelControllerParameters, positionControlledJoints,
                                                                                          controlledOneDofJoints, walkingController, registry);

      registry.addChild(walkingController.getYoVariableRegistry());
   }

   /**
    * Specifies whether the inverse dynamics module of the {@link WholeBodyControllerCore} should be
    * created or not.
    * <p>
    * This module is created by default as the {@link WalkingHighLevelHumanoidController} needs it.
    * </p>
    *
    * @param setup whether to setup the inverse dynamics mode or not.
    */
   public void setupControllerCoreInverseDynamicsMode(boolean setup)
   {
      setupInverseDynamicsSolver = setup;
   }

   /**
    * Specifies whether the inverse kinematics module of the {@link WholeBodyControllerCore} should
    * be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the inverse kinematics mode or not.
    */
   public void setupControllerCoreInverseKinematicsMode(boolean setup)
   {
      setupInverseKinematicsSolver = setup;
   }

   /**
    * Specifies whether the virtual model control module of the {@link WholeBodyControllerCore}
    * should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the virtual model control mode or not.
    */
   public void setupControllerCoreVirtualModelControlMode(boolean setup)
   {
      setupVirtualModelControlSolver = setup;
   }

   public void initialize()
   {
      controllerCore.initialize();
      walkingController.initialize();
   }

   public void initializeDesiredHeightToCurrent()
   {
      walkingController.initializeDesiredHeightToCurrent();
   }

   @Override
   public void doAction()
   {
      walkingController.doAction();
      accelerationIntegrationParameterHelper.update();

      // update the additional joint settings with the values from the acceleration integration parameters
      JointAccelerationIntegrationCommand jointAccelerationIntegrationCommand = accelerationIntegrationParameterHelper.getJointAccelerationIntegrationCommand();
      for (int jointIdx = 0; jointIdx < jointAccelerationIntegrationCommand.getNumberOfJointsToComputeDesiredPositionFor(); jointIdx++)
      {
         OneDoFJoint joint = jointAccelerationIntegrationCommand.getJointToComputeDesiredPositionFor(jointIdx);
         JointAccelerationIntegrationParametersReadOnly jointParameters = jointAccelerationIntegrationCommand.getJointParameters(jointIdx);
         JointDesiredOutput jointDesiredOutput = additionalJointLevelSettings.getJointDesiredOutput(joint);
         jointDesiredOutput.setVelocityIntegrationLeakRate(jointParameters.getAlphaVelocity());
         jointDesiredOutput.setPositionIntegrationLeakRate(jointParameters.getAlphaPosition());
      }

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(jointAccelerationIntegrationCommand);
      controllerCoreCommand.completeLowLevelJointData(additionalJointLevelSettings);

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();
   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      walkingController.reinitializePelvisOrientation(reinitialize);
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
      walkingController.resetJointIntegrators();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      walkingController.resetJointIntegrators();
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    * @return WalkingStateEnum
    */
   public WalkingStateEnum getWalkingStateEnum()
   {
      return walkingController.getWalkingStateEnum();
   }
}