package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;


public class FullyConstrainedState extends AbstractFootControlState
{
   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;
   private final BooleanYoVariable doFancyOnToesControl;

   private final EnumYoVariable<ConstraintType> requestedState;

   private final YoSE3PIDGains gains;
   private final FramePoint2d cop = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   public FullyConstrainedState(RigidBodySpatialAccelerationControlModule accelerationControlModule, MomentumBasedController momentumBasedController,
         ContactablePlaneBody contactableBody, BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape, PartialFootholdControlModule partialFootholdControlModule,
         FrameVector fullyConstrainedNormalContactVector, BooleanYoVariable doFancyOnToesControl, YoSE3PIDGains gains, RobotSide robotSide, YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, accelerationControlModule, momentumBasedController,
            contactableBody, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, robotSide, registry);

      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
      this.doFancyOnToesControl = doFancyOnToesControl;
      this.requestedState = requestedState;
      this.gains = gains;
      this.partialFootholdControlModule = partialFootholdControlModule;
   }

   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, fullyConstrainedNormalContactVector);
   }

   private void setFullyConstrainedStateGains()
   {
      accelerationControlModule.setGains(gains);
   }

   public void doSpecificAction()
   {
      if (FootControlModule.USE_AUTOMATIC_FOOT_SHRINK)
      {
         momentumBasedController.getFootSwitches().get(robotSide).computeAndPackCoP(cop);
         FramePoint2d desiredCoP = momentumBasedController.getCoP(contactableBody);
         partialFootholdControlModule.compute(desiredCoP, cop);
         YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
         partialFootholdControlModule.applyShrunkPolygon(contactState);
      }

      if (doFancyOnToesControl.getBooleanValue())
         determineCoPOnEdge();

      if (FootControlModule.USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
      {
         if (isCoPOnEdge && doFancyOnToesControl.getBooleanValue())
            requestedState.set(ConstraintType.HOLD_POSITION);
         else if (requestHoldPosition != null && requestHoldPosition.getBooleanValue())
            requestedState.set(ConstraintType.HOLD_POSITION);
      }

      if (gains == null)
      {
         footAcceleration.setToZero(contactableBody.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableBody.getFrameAfterParentJoint());
      }
      else
      {
         setFullyConstrainedStateGains();

         desiredPosition.setToZero(contactableBody.getFrameAfterParentJoint());
         desiredPosition.changeFrame(worldFrame);

         desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
         desiredOrientation.changeFrame(worldFrame);

         desiredLinearVelocity.setToZero(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         desiredLinearAcceleration.setToZero(worldFrame);
         desiredAngularAcceleration.setToZero(worldFrame);

         accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
         accelerationControlModule.packAcceleration(footAcceleration);
      }

      setTaskspaceConstraint(footAcceleration);
   }
}
