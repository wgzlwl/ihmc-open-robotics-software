package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedPlanarFootstepPlan;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.*;

public class QuadrupedXGaitPlanner
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final QuadrantDependentList<FramePoint3D> xGaitRectangleVertices;
   private final FramePose3D xGaitRectanglePose;
   private final PoseReferenceFrame xGaitRectangleFrame;
   private final EndDependentList<QuadrupedTimedStep> pastSteps;

   public QuadrupedXGaitPlanner()
   {
      xGaitRectangleVertices = new QuadrantDependentList<>();
      xGaitRectanglePose = new FramePose3D(worldFrame);
      xGaitRectangleFrame = new PoseReferenceFrame("xGaitRectangleFrame", worldFrame);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangleVertices.set(robotQuadrant, new FramePoint3D(xGaitRectangleFrame));
      }
      pastSteps = new EndDependentList<>();
      pastSteps.put(RobotEnd.FRONT, new QuadrupedTimedStep());
      pastSteps.put(RobotEnd.HIND, new QuadrupedTimedStep());
   }

   // fixme this is getting initialized incorrectly
   public void computeInitialPlan(QuadrupedPlanarFootstepPlan footstepPlan, Vector3D planarVelocity, RobotQuadrant initialStepQuadrant,
                                  FramePoint3D currentSupportCentroid, double currentTime, double currentYaw, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      currentSupportCentroid.changeFrame(worldFrame);
      xGaitRectanglePose.setPosition(currentSupportCentroid);
      xGaitRectanglePose.setOrientationYawPitchRoll(currentYaw, 0, 0);
      xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangleVertices.get(robotQuadrant).changeFrame(xGaitRectangleFrame);
         xGaitRectangleVertices.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangleVertices.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
         xGaitRectangleVertices.get(robotQuadrant).setZ(0);
      }

      // plan steps
      double lastStepStartTime = currentTime;
      RobotQuadrant lastStepQuadrant = initialStepQuadrant.getNextReversedRegularGaitSwingQuadrant();
      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = footstepPlan.getPlannedSteps();
      plannedSteps.clear();
      for (int i = 0; i < plannedSteps.capacity(); i++)
      {
         plannedSteps.add();
         QuadrupedTimedOrientedStep step = plannedSteps.get(plannedSteps.size() - 1);

         // compute step quadrant
         RobotQuadrant thisStepQuadrant = lastStepQuadrant.getNextRegularGaitSwingQuadrant();
         step.setRobotQuadrant(thisStepQuadrant);

         // compute step timing
         double thisStepStartTime;
         double thisStepEndTime;
         if (i == 0)
         {
            thisStepStartTime = currentTime;
            thisStepEndTime = currentTime + xGaitSettings.getStepDuration();
         }
         else
         {
            double endPhaseShift = thisStepQuadrant.isQuadrantInHind() ? 180.0 - xGaitSettings.getEndPhaseShift() : xGaitSettings.getEndPhaseShift();
            double endTimeShift = xGaitSettings.getEndDoubleSupportDuration() + xGaitSettings.getStepDuration();
            endTimeShift *= Math.max(Math.min(endPhaseShift, 180.0), 0.0) / 180.0;
            thisStepStartTime = lastStepStartTime + endTimeShift;
            thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();
         }
         step.getTimeInterval().setStartTime(thisStepStartTime);
         step.getTimeInterval().setEndTime(thisStepEndTime);

         // compute xGait rectangle pose at end of step
         double deltaTime = thisStepEndTime - currentTime;
         extrapolatePose(xGaitRectanglePose, planarVelocity, deltaTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);
         step.setStepYaw(xGaitRectanglePose.getYaw());

         // compute step goal position by sampling the corner position of the xGait rectangle at touch down
         RobotQuadrant robotQuadrant = step.getRobotQuadrant();
         step.setGoalPosition(xGaitRectangleVertices.get(robotQuadrant));

         // compute step ground clearance
         step.setGroundClearance(xGaitSettings.getStepGroundClearance());

         // update state for next step
         lastStepStartTime = thisStepStartTime;
         lastStepQuadrant = thisStepQuadrant;
      }
   }

   public void computeOnlinePlan(QuadrupedPlanarFootstepPlan footstepPlan, Vector3D desiredVelocity, ReferenceFrame supportFrame, double currentTime,
                                 double currentYaw, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      // initialize latest step
      QuadrupedTimedStep latestStep;
      EndDependentList<QuadrupedTimedOrientedStep> currentSteps = footstepPlan.getCurrentSteps();

      if (currentSteps.get(RobotEnd.HIND).getTimeInterval().getEndTime() > currentSteps.get(RobotEnd.FRONT).getTimeInterval().getEndTime())
         latestStep = currentSteps.get(RobotEnd.HIND);
      else
         latestStep = currentSteps.get(RobotEnd.FRONT);

      xGaitRectanglePose.setToZero(supportFrame);
      xGaitRectanglePose.changeFrame(worldFrame);
      xGaitRectanglePose.setOrientationYawPitchRoll(currentYaw, 0, 0);
      xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

      // initialize nominal support rectangle
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         xGaitRectangleVertices.get(robotQuadrant).setToZero(xGaitRectangleFrame);
         xGaitRectangleVertices.get(robotQuadrant).setX(robotQuadrant.getEnd().negateIfHindEnd(xGaitSettings.getStanceLength() / 2.0));
         xGaitRectangleVertices.get(robotQuadrant).setY(robotQuadrant.getSide().negateIfRightSide(xGaitSettings.getStanceWidth() / 2.0));
      }

      PreallocatedList<QuadrupedTimedOrientedStep> steps = footstepPlan.getPlannedSteps();
      steps.clear();

      // compute step quadrants and time intervals
      RobotEnd thisStepEnd = latestStep.getRobotQuadrant().getOppositeEnd();
      pastSteps.set(RobotEnd.FRONT, currentSteps.get(RobotEnd.FRONT));
      pastSteps.set(RobotEnd.HIND, currentSteps.get(RobotEnd.HIND));

      for (int i = 0; i < steps.capacity(); i++)
      {
         steps.add();
         QuadrupedTimedStep thisStep = steps.get(i);
         QuadrupedTimedStep pastStepOnSameEnd = pastSteps.get(thisStepEnd);
         QuadrupedTimedStep pastStepOnOppositeEnd = pastSteps.get(thisStepEnd.getOppositeEnd());

         thisStep.setRobotQuadrant(pastStepOnSameEnd.getRobotQuadrant().getAcrossBodyQuadrant());
         computeStepTimeInterval(thisStep, pastStepOnSameEnd, pastStepOnOppositeEnd, xGaitSettings);
         if (currentTime > thisStep.getTimeInterval().getStartTime())
            thisStep.getTimeInterval().shiftInterval(currentTime - thisStep.getTimeInterval().getStartTime());

         pastSteps.set(thisStepEnd, thisStep);
         thisStepEnd = thisStepEnd.getOppositeEnd();
      }

      double localTime = currentTime;

      // fixme this has a bug when walking, and doesn't account well for transfer
      // compute step goal positions and ground clearances
      for (int i = 0; i < steps.size(); i++)
      {
         // compute xGait rectangle pose at end of step
         double deltaTime = steps.get(i).getTimeInterval().getEndTime() - localTime;
         extrapolatePose(xGaitRectanglePose, desiredVelocity, deltaTime);
         xGaitRectangleFrame.setPoseAndUpdate(xGaitRectanglePose);

         // compute step goal position by sampling the corner position of the xGait rectangle at touchdown
         RobotQuadrant stepQuadrant = steps.get(i).getRobotQuadrant();
         steps.get(i).setGoalPosition(xGaitRectangleVertices.get(stepQuadrant));
         steps.get(i).setStepYaw(xGaitRectanglePose.getYaw());

         // compute step ground clearance
         steps.get(i).setGroundClearance(xGaitSettings.getStepGroundClearance());

         localTime = steps.get(i).getTimeInterval().getEndTime();
      }
   }

   private final Vector3D translation = new Vector3D();

   private void extrapolatePose(FramePose3D poseToExtrapolateToPack, Vector3DReadOnly planarVelocity, double deltaTime)
   {
      double currentYaw = poseToExtrapolateToPack.getYaw();

      // initialize forward, lateral, and rotational velocity in pose frame
      double forwardVelocity = planarVelocity.getX();
      double lateralVelocity = planarVelocity.getY();
      double rotationalVelocity = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double yawRotation;
      double epsilon = 0.001;
      if (MathTools.epsilonEquals(rotationalVelocity, 0.0, epsilon))
      {
         yawRotation = 0.0;
         translation.setX((forwardVelocity * Math.cos(yawRotation) - lateralVelocity * Math.sin(yawRotation)) * deltaTime);
         translation.setY((forwardVelocity * Math.sin(yawRotation) + lateralVelocity * Math.cos(yawRotation)) * deltaTime);
      }
      else
      {
         yawRotation = rotationalVelocity * deltaTime;
         translation.setX(forwardVelocity / rotationalVelocity * (Math.sin(yawRotation) - Math.sin(currentYaw)) + lateralVelocity / rotationalVelocity * (
               Math.cos(yawRotation) - Math.cos(currentYaw)));
         translation.setY(-forwardVelocity / rotationalVelocity * (Math.cos(yawRotation) - Math.cos(currentYaw)) + lateralVelocity / rotationalVelocity * (
               Math.sin(yawRotation) - Math.sin(currentYaw)));
      }

      poseToExtrapolateToPack.appendTranslation(translation);
      poseToExtrapolateToPack.appendYawRotation(yawRotation);
   }

   private void computeStepTimeInterval(QuadrupedTimedStep thisStep, QuadrupedTimedStep pastStepOnSameEnd, QuadrupedTimedStep pastStepOnOppositeEnd,
                                        QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      RobotEnd thisStepEnd = thisStep.getRobotQuadrant().getEnd();
      RobotSide thisStepSide = thisStep.getRobotQuadrant().getSide();
      RobotSide pastStepSide = pastStepOnOppositeEnd.getRobotQuadrant().getSide();

      double pastStepEndTimeForSameEnd = pastStepOnSameEnd.getTimeInterval().getEndTime();
      double pastStepEndTimeForOppositeEnd = pastStepOnOppositeEnd.getTimeInterval().getEndTime();

      // Compute support durations and end phase shift.
      double nominalStepDuration = xGaitSettings.getStepDuration();
      double endDoubleSupportDuration = xGaitSettings.getEndDoubleSupportDuration();
      double endPhaseShift = MathTools.clamp(xGaitSettings.getEndPhaseShift(), 0, 359);
      if (thisStepEnd == RobotEnd.HIND)
         endPhaseShift = 360 - endPhaseShift;
      if (pastStepSide != thisStepSide)
         endPhaseShift = endPhaseShift - 180;

      // Compute step time interval. Step duration is scaled in the range (1.0, 1.5) to account for end phase shifts.
      double thisStepStartTime = pastStepEndTimeForSameEnd + endDoubleSupportDuration;
      double thisStepEndTime = pastStepEndTimeForOppositeEnd + (nominalStepDuration + endDoubleSupportDuration) * endPhaseShift / 180.0;
      double thisStepDuration = MathTools.clamp(thisStepEndTime - thisStepStartTime, nominalStepDuration, 1.5 * nominalStepDuration);

      thisStep.getTimeInterval().setStartTime(thisStepStartTime);
      thisStep.getTimeInterval().setEndTime(thisStepStartTime + thisStepDuration);
   }
}