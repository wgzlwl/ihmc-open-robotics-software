package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CentroidProjectionToeOffCalculator implements ToeOffCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final String namePrefix = "centProj";

   private final SideDependentList<List<YoContactPoint>> contactPoints = new SideDependentList<>();

   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FrameLineSegment2d toeOffContactLine2d = new FrameLineSegment2d();

   private final FramePoint exitCMP = new FramePoint();
   private final FramePoint2d exitCMP2d = new FramePoint2d();
   private final FrameVector2d exitCMPRayDirection2d = new FrameVector2d();

   private final FramePoint2d tmpPoint2d = new FramePoint2d();
   private final FrameLine2d rayThroughExitCMP = new FrameLine2d();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();

   private final DoubleYoVariable toeOffContactInterpolation;
   private final BooleanYoVariable hasComputedToeOffContactPoint;
   private final BooleanYoVariable hasComputedToeOffContactLine;

   private final FramePoint2d[] intersectionWithRay = new FramePoint2d[] {new FramePoint2d(), new FramePoint2d()};

   public CentroidProjectionToeOffCalculator(SideDependentList<YoPlaneContactState> contactStates, SideDependentList<? extends ContactablePlaneBody> feet,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         contactPoints.put(robotSide, contactStates.get(robotSide).getContactPoints());
      }

      toeOffContactInterpolation = new DoubleYoVariable(namePrefix + "ToeOffContactInterpolation", registry);
      toeOffContactInterpolation.set(walkingControllerParameters.getToeOffContactInterpolation());

      hasComputedToeOffContactPoint = new BooleanYoVariable(namePrefix + "HasComputedToeOffContactPoint", registry);
      hasComputedToeOffContactLine = new BooleanYoVariable(namePrefix + "HasComputedToeOffContactLine", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public ToeOffEnum getEnum()
   {
      return ToeOffEnum.CENTROID_PROJECTION;
   }

   @Override
   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
      hasComputedToeOffContactLine.set(false);
   }

   @Override
   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setToZero(soleFrame);
      exitCMP2d.setByProjectionOntoXYPlaneIncludingFrame(this.exitCMP);
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);

      computeFootPolygon(trailingLeg);

      FramePoint2d rayOrigin;
      if (!exitCMP2d.containsNaN() && footPolygon.isPointInside(exitCMP2d))
         rayOrigin = exitCMP2d;
      else
         rayOrigin = footPolygon.getCentroid();

      exitCMPRayDirection2d.setIncludingFrame(soleFrame, 1.0, 0.0);
      rayThroughExitCMP.setToZero(soleFrame);

      if (desiredCMP != null && !desiredCMP.containsNaN())
      {
         tmpPoint2d.setToZero(soleFrame);
         desiredCMP.changeFrameAndProjectToXYPlane(soleFrame);
         tmpPoint2d.interpolate(rayOrigin, desiredCMP, toeOffContactInterpolation.getDoubleValue());

         if (footPolygon.isPointInside(tmpPoint2d))
            rayThroughExitCMP.set(tmpPoint2d, exitCMPRayDirection2d);
         else
            rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }
      else
      {
         rayThroughExitCMP.set(rayOrigin, exitCMPRayDirection2d);
      }

      footPolygon.intersectionWithRay(rayThroughExitCMP, intersectionWithRay[0], intersectionWithRay[1]);
      toeOffContactPoint2d.set(intersectionWithRay[0]);

      hasComputedToeOffContactPoint.set(true);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      computeFootPolygon(trailingLeg);

      ReferenceFrame referenceFrame = footPolygon.getReferenceFrame();
      toeOffContactLine2d.setToZero(referenceFrame);
      toeOffContactLine2d.setFirstEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
      toeOffContactLine2d.setSecondEndpoint(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

      // gets the leading two toe points
      for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
      {
         footPolygon.getFrameVertex(i, tmpPoint2d);
         if (tmpPoint2d.getX() > toeOffContactLine2d.getFirstEndpoint().getX())
         { // further ahead than leading point
            toeOffContactLine2d.flipDirection();
            toeOffContactLine2d.setFirstEndpoint(tmpPoint2d);
         }
         else if (tmpPoint2d.getX() > toeOffContactLine2d.getSecondEndpoint().getX())
         { // further ahead than second leading point
            toeOffContactLine2d.setSecondEndpoint(tmpPoint2d);
         }
      }

      hasComputedToeOffContactLine.set(true);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactLine.getBooleanValue())
         computeToeOffContactLine(null, trailingLeg);

      contactLineToPack.set(toeOffContactLine2d);
   }

   private void computeFootPolygon(RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      footPolygon.clear(soleFrame);
      for (int i = 0; i < contactPoints.get(trailingLeg).size(); i++)
      {
         contactPoints.get(trailingLeg).get(i).getPosition2d(toeOffContactPoint2d);
         footPolygon.addVertex(toeOffContactPoint2d);
      }
      footPolygon.update();
   }
}