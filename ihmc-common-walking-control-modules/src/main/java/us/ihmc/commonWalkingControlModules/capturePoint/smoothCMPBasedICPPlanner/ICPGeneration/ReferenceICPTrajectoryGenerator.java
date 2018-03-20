package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Tim Seyde
 */

public class ReferenceICPTrajectoryGenerator implements TrajectoryGenerator
{
   private final static boolean CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY = true;

   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 50;

   private final boolean debug;

   private final List<FramePoint3D> icpDesiredInitialPositions = new ArrayList<>();
   private final List<FramePoint3D> icpDesiredFinalPositions = new ArrayList<>();

   private final List<FramePoint3D> icpDesiredInitialPositionsFromCoPs = new ArrayList<>();
   private final List<FramePoint3D> icpDesiredFinalPositionsFromCoPs = new ArrayList<>();

   private final FramePoint3D desiredICPPosition = new FramePoint3D();
   private final FrameVector3D desiredICPVelocity = new FrameVector3D();
   private final FrameVector3D desiredICPAcceleration = new FrameVector3D();
   private final FrameVector3D icpVelocityDynamicsCurrent = new FrameVector3D();
   private final YoFrameVector yoICPVelocityDynamicsCurrent;

   private final FramePoint3D icpPositionDesiredFinalCurrentSegment = new FramePoint3D();
   private final FramePoint3D icpPositionDesiredTerminal = new FramePoint3D();

   private final YoBoolean isInitialTransfer;

   private final YoBoolean continuouslyAdjustForICPContinuity;
   private final YoBoolean areICPDynamicsSatisfied;

   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfFootstepsToConsider;
   private final YoInteger currentSegmentIndex;

   private final YoDouble startTimeOfCurrentPhase;
   private final YoDouble localTimeInCurrentPhase;
   private final YoDouble omega0;

   private int numberOfFootstepsRegistered;

   private final List<FrameTrajectory3D> copTrajectories = new ArrayList<>();
   private final List<FrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   private final TIntArrayList icpPhaseExitCornerPointIndices = new TIntArrayList();
   private final TIntArrayList icpPhaseEntryCornerPointIndices = new TIntArrayList();

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final List<FrameTuple3DBasics> icpInitialConditions = new ArrayList<>();

   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox(icpToolbox);

   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isInitialTransfer, boolean debug,
                                          YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isInitialTransfer = isInitialTransfer;
      this.debug = debug;

      areICPDynamicsSatisfied = new YoBoolean(namePrefix + "AreICPDynamicsSatisfied", registry);
      areICPDynamicsSatisfied.set(false);

      continuouslyAdjustForICPContinuity = new YoBoolean(namePrefix + "ContinuouslyAdjustForICPContinuity", registry);
      continuouslyAdjustForICPContinuity.set(CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY);

      totalNumberOfCMPSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);

      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);

      yoICPVelocityDynamicsCurrent = new YoFrameVector(namePrefix + "ICPVelocityDynamics", ReferenceFrame.getWorldFrame(), registry);

      currentSegmentIndex = new YoInteger(namePrefix + "CurrentSegment", registry);

      for (int i = 0; i < defaultSize; i++)
      {
         icpDesiredInitialPositions.add(new FramePoint3D());
         icpDesiredFinalPositions.add(new FramePoint3D());

         icpDesiredInitialPositionsFromCoPs.add(new FramePoint3D());
         icpDesiredFinalPositionsFromCoPs.add(new FramePoint3D());
      }

      icpInitialConditions.add(new FramePoint3D());
      icpInitialConditions.add(new FrameVector3D());
      icpInitialConditions.add(new FrameVector3D());
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfCMPSegments.set(0);
      localTimeInCurrentPhase.set(0.0);

      icpPhaseEntryCornerPointIndices.resetQuick();
      icpPhaseExitCornerPointIndices.resetQuick();
   }

   private void resetCoPs()
   {
      copTrajectories.clear();
   }

   public void initializeForTransfer(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                     List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      initialize();
   }

   public void initializeForSwing(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                  List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }
      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      initialize();
   }

   public void initializeForTransferFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                             List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         int copSegments = transferCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
         }

         SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copSegments = swingCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
         }
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      int copSegments = transferCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
      }

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());

      if (isInitialTransfer.getBooleanValue())
      {
         FrameTrajectory3D copPolynomial3D = copTrajectories.get(0);
         copPolynomial3D.getDerivative(0, localTimeInCurrentPhase.getDoubleValue(), desiredICPPosition);
         copPolynomial3D.getDerivative(1, localTimeInCurrentPhase.getDoubleValue(), desiredICPVelocity);
         copPolynomial3D.getDerivative(2, localTimeInCurrentPhase.getDoubleValue(), desiredICPAcceleration);

         setInitialConditionsForAdjustment(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);
      }
   }

   public void initializeForSwingFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                          List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(0);
      int copSegments = swingCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         copSegments = transferCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
         }

         swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copSegments = swingCoPTrajectory.getNumberOfSegments();
         for (int copSegment = 0; copSegment < copSegments; copSegment++)
         {
            copTrajectories.add(swingCoPTrajectory.getSegment(copSegment));
         }
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      copSegments = transferCoPTrajectory.getNumberOfSegments();
      for (int copSegment = 0; copSegment < copSegments; copSegment++)
      {
         copTrajectories.add(transferCoPTrajectory.getSegment(copSegment));
      }

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
   }

   @Override
   public void initialize()
   {
      if (isInitialTransfer.getBooleanValue())
      {
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(0);
         cmpPolynomial3D.compute(cmpPolynomial3D.getInitialTime());
      }

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());

      icpPositionDesiredTerminal.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));
   }

   public void setInitialConditionsForAdjustment(FramePoint3DReadOnly desiredICPPosition, FrameVector3DReadOnly desiredICPVelocity,
                                                 FrameVector3DReadOnly desiredICPAcceleration)
   {
      icpInitialConditions.get(0).setIncludingFrame(desiredICPPosition);
      icpInitialConditions.get(1).setIncludingFrame(desiredICPVelocity);
      icpInitialConditions.get(2).setIncludingFrame(desiredICPAcceleration);
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing()
   {
      if ((isInitialTransfer.getBooleanValue() || (continuouslyAdjustForICPContinuity.getBooleanValue())) && copTrajectories.size() > 1)
      {
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0.getDoubleValue(), copTrajectories, icpInitialConditions,
                                                                             icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs);

      }
   }

   @Override
   public void compute(double time)
   {
      if (cmpTrajectories.size() > 0)
      {
         localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());

         currentSegmentIndex.set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), cmpTrajectories));
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentSegmentIndex.getIntegerValue());
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());

         // ICP
         icpToolbox.computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                       cmpPolynomial3D, desiredICPPosition);
         icpToolbox.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                       cmpPolynomial3D, desiredICPVelocity);
         icpToolbox
               .computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                       cmpPolynomial3D, desiredICPAcceleration);

         if (debug)
            checkICPDynamics(localTimeInCurrentPhase.getDoubleValue(), desiredICPVelocity, desiredICPPosition, cmpPolynomial3D);
      }
   }

   private void checkICPDynamics(double time, FrameVector3D icpVelocityDesiredCurrent, FramePoint3D icpPositionDesiredCurrent,
                                 FrameTrajectory3D cmpPolynomial3D)
   {
      cmpPolynomial3D.compute(time);

      icpVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, cmpPolynomial3D.getFramePosition());
      icpVelocityDynamicsCurrent.scale(omega0.getDoubleValue());

      yoICPVelocityDynamicsCurrent.set(icpVelocityDynamicsCurrent);

      areICPDynamicsSatisfied.set(icpVelocityDesiredCurrent.epsilonEquals(icpVelocityDynamicsCurrent, 10e-5));
   }

   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<FrameTrajectory3D> cmpTrajectories)
   {
      int currentSegmentIndex = FIRST_SEGMENT;
      while (timeInCurrentPhase > cmpTrajectories.get(currentSegmentIndex).getFinalTime()
            && Math.abs(cmpTrajectories.get(currentSegmentIndex).getFinalTime() - cmpTrajectories.get(currentSegmentIndex + 1).getInitialTime()) < 1.0e-5)
      {
         currentSegmentIndex++;
         if (currentSegmentIndex + 1 > cmpTrajectories.size())
         {
            return currentSegmentIndex;
         }
      }
      return currentSegmentIndex;
   }

   public void getICPPositionDesiredFinalFromSegment(FramePoint3D icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
   }

   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.set(desiredICPPosition);
   }

   public void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      velocityToPack.set(desiredICPVelocity);
   }

   public void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.set(desiredICPAcceleration);
   }

   public void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack, FixedFrameVector3DBasics accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   public List<? extends FramePoint3DReadOnly> getICPPositionDesiredInitialList()
   {
      return icpDesiredInitialPositions;
   }

   public List<? extends FramePoint3DReadOnly> getICPPositionDesiredFinalList()
   {
      return icpDesiredFinalPositions;
   }

   public void getICPPhaseEntryCornerPoints(List<? extends FixedFramePoint3DBasics> icpPhaseEntryCornerPointsToPack)
   {
      int i = 0;
      for (; i < icpPhaseEntryCornerPointIndices.size(); i++)
      {
         FramePoint3D icpPhaseEntryCornerPoint = icpDesiredInitialPositions.get(icpPhaseEntryCornerPointIndices.get(i));
         icpPhaseEntryCornerPointsToPack.get(i).set(icpPhaseEntryCornerPoint);
         icpPhaseEntryCornerPointsToPack.get(i).add(0.0, 0.0, 0.03);
      }
      for (; i < icpPhaseEntryCornerPointsToPack.size(); i++)
         icpPhaseEntryCornerPointsToPack.get(i).setToNaN();

   }

   public void getICPPhaseExitCornerPoints(List<? extends FixedFramePoint3DBasics> icpPhaseExitCornerPointsToPack)
   {
      int i = 0;
      for (; i < icpPhaseExitCornerPointIndices.size(); i++)
      {
         FramePoint3D icpPhaseExitCornerPoint = icpDesiredFinalPositions.get(icpPhaseExitCornerPointIndices.get(i) - 1);
         icpPhaseExitCornerPointsToPack.get(i).set(icpPhaseExitCornerPoint);
         icpPhaseExitCornerPointsToPack.get(i).add(0.0, 0.0, 0.05);
      }
      for (; i < icpPhaseExitCornerPointsToPack.size(); i++)
         icpPhaseExitCornerPointsToPack.get(i).setToNaN();
   }

   public List<? extends FramePoint3DReadOnly> getICPPositionFromCoPDesiredFinalList()
   {
      return icpDesiredFinalPositionsFromCoPs;
   }
}
