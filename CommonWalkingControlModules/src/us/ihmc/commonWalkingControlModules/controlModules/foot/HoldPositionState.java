package us.ihmc.commonWalkingControlModules.controlModules.foot;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FrameVector;

public class HoldPositionState extends FootControlState
{
   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;
   
   public HoldPositionState(YoFramePoint yoDesiredPosition,
         YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody,
         BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, FrameVector fullyConstrainedNormalContactVector,
         BooleanYoVariable forceFootAccelerateIntoGround, LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule)
   {
      super(ConstraintType.HOLD_POSITION, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
            accelerationControlModule, momentumBasedController, contactableBody, requestedState,
            jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape,
            forceFootAccelerateIntoGround, legSingularityAndKneeCollapseAvoidanceControlModule);
   
      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
   }

   public void doTransitionIntoAction()
   {
      // Remember the previous contact normal, in case the foot leaves the ground and rotates
      holdPositionNormalContactVector.setIncludingFrame(fullyConstrainedNormalContactVector);
      holdPositionNormalContactVector.changeFrame(worldFrame);
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, holdPositionNormalContactVector);

      desiredPosition.setToZero(contactableBody.getBodyFrame());
      desiredPosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(contactableBody.getBodyFrame());
      desiredOrientation.changeFrame(worldFrame);
      
      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);
   }

   public void doSpecificAction()
   {
      determineCoPOnEdge();

      if (!isCoPOnEdge && (requestHoldPosition == null || !requestHoldPosition.getBooleanValue()))
         requestedState.set(ConstraintType.FULL);
      
      yoDesiredPosition.set(desiredPosition);
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      if (forceFootAccelerateIntoGround.getBooleanValue())
         footAcceleration.setLinearPartZ(footAcceleration.getLinearPartZ() + desiredZAccelerationIntoGround);
      
      setTaskspaceConstraint(footAcceleration);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      yoDesiredPosition.setToNaN();
   }
   
   public void setHoldPositionStateGains(double holdZeta, double holdKpx, double holdKpy,
         double holdKpz, double holdKdz, double holdKpRoll, double holdKpPitch, double holdKpYaw)
   {
      double dxPosition = GainCalculator.computeDerivativeGain(holdKpx, holdZeta);
      double dyPosition = GainCalculator.computeDerivativeGain(holdKpy, holdZeta);
      double dzPosition = GainCalculator.computeDerivativeGain(holdKpz, holdZeta);
      double dxOrientation = GainCalculator.computeDerivativeGain(holdKpRoll, holdZeta);
      double dyOrientation = GainCalculator.computeDerivativeGain(holdKpPitch, holdZeta);
      double dzOrientation = GainCalculator.computeDerivativeGain(holdKpYaw, holdZeta);

      accelerationControlModule.setPositionProportionalGains(holdKpx, holdKpy, holdKpz);
      accelerationControlModule.setPositionDerivativeGains(dxPosition, dyPosition, dzPosition);
      accelerationControlModule.setOrientationProportionalGains(holdKpRoll, holdKpPitch, holdKpYaw);
      accelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }
}
