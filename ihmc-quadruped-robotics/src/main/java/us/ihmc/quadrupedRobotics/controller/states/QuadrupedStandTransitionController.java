package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedStandTransitionController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleParameter blendingDuration = new DoubleParameter("blendingDuration", registry, 1.0);
   private final YoDouble blendingFactor = new YoDouble("standTransitionBlendingFactor", registry);

   private final QuadrupedController steppingState;
   private final QuadrupedController freezeController;

   private final JointDesiredOutputList jointDesiredOutputList;

   private final OneDoFJoint[] controllerJoints;

   public QuadrupedStandTransitionController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedController steppingState,
                                             QuadrupedController freezeController, YoVariableRegistry parentRegistry)
   {
      this.controllerJoints = runtimeEnvironment.getFullRobotModel().getControllableOneDoFJoints();
      this.steppingState = steppingState;
      this.freezeController = freezeController;
      this.jointDesiredOutputList = new JointDesiredOutputList(controllerJoints);

      parentRegistry.addChild(registry);
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      if (blendingFactor.getDoubleValue() >= 1.0)
         return ControllerEvent.DONE;
      else
         return null;
   }

   @Override
   public void onEntry()
   {
      steppingState.onEntry();
   }

   @Override
   public void doAction(double timeInState)
   {
      freezeController.doAction(timeInState);
      steppingState.doAction(timeInState);

      blendingFactor.set(timeInState / blendingDuration.getValue());

      for (OneDoFJoint controlledJoint : controllerJoints)
      {
         JointDesiredOutputReadOnly freezeCommand = freezeController.getJointDesiredOutputList().getJointDesiredOutput(controlledJoint);
         JointDesiredOutputReadOnly balanceCommand = steppingState.getJointDesiredOutputList().getJointDesiredOutput(controlledJoint);

         double desiredPosition = freezeCommand.getDesiredPosition();
         double currentPosition = controlledJoint.getQ();

         double modifiedDesiredTorque = blendingFactor.getDoubleValue() * balanceCommand.getDesiredTorque();
         double modifiedDesiredPosition = (1.0 - blendingFactor.getDoubleValue()) * desiredPosition + blendingFactor.getDoubleValue() * currentPosition;

         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(controlledJoint);
         jointDesiredOutput.setControlMode(balanceCommand.getControlMode());
         jointDesiredOutput.setDesiredTorque(modifiedDesiredTorque);
         jointDesiredOutput.setDesiredPosition(modifiedDesiredPosition);
         jointDesiredOutput.setStiffness(freezeCommand.getStiffness());
         jointDesiredOutput.setDamping(freezeCommand.getDamping());
      }
   }

   @Override
   public void onExit()
   {
      freezeController.onExit();
   }

   @Override
   public JointDesiredOutputList getJointDesiredOutputList()
   {
      return jointDesiredOutputList;
   }
}
