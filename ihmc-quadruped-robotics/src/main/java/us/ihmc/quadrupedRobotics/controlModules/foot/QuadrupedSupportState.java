package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSupportState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final YoPlaneContactState contactState;

   private final SpatialVelocityCommand spatialVelocityCommand = new SpatialVelocityCommand();
   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);

   private final ReferenceFrame soleFrame;

   public QuadrupedSupportState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox)
   {
      this.robotQuadrant = robotQuadrant;
      this.contactState = controllerToolbox.getFootContactState(robotQuadrant);

      soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);

      spatialVelocityCommand.set(controllerToolbox.getFullRobotModel().getElevator(), controllerToolbox.getFullRobotModel().getFoot(robotQuadrant));
      spatialVelocityCommand.setSelectionMatrixForLinearControl();
      spatialVelocityCommand.setWeight(10.0);
   }

   @Override
   public void onEntry()
   {
      contactState.setFullyConstrained();
      contactState.setContactNormalVector(footNormalContactVector);

      if (waypointCallback != null)
         waypointCallback.isDoneMoving(robotQuadrant, true);
   }

   @Override
   public void doAction(double timeInState)
   {
      spatialVelocityCommand.setSpatialVelocityToZero(soleFrame);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public InverseKinematicsCommand<?> getInverseKinematicsCommand()
   {
//      return null;
      return spatialVelocityCommand;
   }


   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }
}
