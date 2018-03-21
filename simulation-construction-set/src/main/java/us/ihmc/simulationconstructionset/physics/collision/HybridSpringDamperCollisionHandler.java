package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ContactingExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeWithLink;
import us.ihmc.simulationconstructionset.physics.Contacts;

public class HybridSpringDamperCollisionHandler implements CollisionHandler
{
   private final Random random;
   private final double coefficientOfFriction;
   private final double coefficientOfRestitution;

   private final double rotationalCoefficientOfFrictionBeta;
   private final double kpCollision;
   private final double kdCollision;
   private final double kdRotationalDamping;
   private final double pullingOutSpringHysteresisReduction;

   private List<CollisionHandlerListener> listeners = new ArrayList<CollisionHandlerListener>();

   //
   private static final boolean performSpringDamper = true;
   private static final boolean slipTowardEachOtherIfSlipping = true;
   private static final boolean createNewContactPairs = true;

   private static final boolean allowRecyclingOfPointsInUse = true;

   private static final boolean useAverageNewCollisionTouchdownPoints = true;

   private static final boolean divideByNumberContacting = true;

   private final ArrayList<Contacts> shapesInContactList = new ArrayList<Contacts>();

   private final ArrayList<ContactingExternalForcePoint> allContactingExternalForcePoints = new ArrayList<>();
   
   public HybridSpringDamperCollisionHandler(double epsilon, double mu)
   {
      this(new Random(), epsilon, mu);
   }

   public HybridSpringDamperCollisionHandler(Random random, double epsilon, double mu)
   {
      this.random = random;
      this.coefficientOfRestitution = epsilon;
      this.coefficientOfFriction = mu;

      rotationalCoefficientOfFrictionBeta = 0.01;
      kpCollision = 2000.0;
      kdCollision = 2000.0;
      kdRotationalDamping = 0.05;
      pullingOutSpringHysteresisReduction = 0.8;
   }

   @Override
   public void maintenanceBeforeCollisionDetection()
   {
      shapesInContactList.clear();

   }

   @Override
   public void maintenanceAfterCollisionDetection()
   {
      int numberOfCollisions = shapesInContactList.size();

      Collections.shuffle(shapesInContactList, random);

      for (int i = 0; i < numberOfCollisions; i++)
      {
         Contacts shapesInContact = shapesInContactList.get(i);

         //TODO: Get rid of Type cast here...
         CollisionShapeWithLink shape1 = (CollisionShapeWithLink) shapesInContact.getShapeA();
         CollisionShapeWithLink shape2 = (CollisionShapeWithLink) shapesInContact.getShapeB();
         handleLocal(shape1, shape2, shapesInContact);
      }

      for (int index = 0; index < allContactingExternalForcePoints.size(); index++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = allContactingExternalForcePoints.get(index);
         contactingExternalForcePointOne.setForce(0.0, 0.0, 0.0);
         contactingExternalForcePointOne.setImpulse(0.0, 0.0, 0.0);
         contactingExternalForcePointOne.setMoment(0.0, 0.0, 0.0);

      }

      for (int index = 0; index < allContactingExternalForcePoints.size(); index++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = allContactingExternalForcePoints.get(index);

         int indexOfContactingPair = contactingExternalForcePointOne.getIndexOfContactingPair();
         if (indexOfContactingPair == -1)
         {
            continue;
         }

         ContactingExternalForcePoint contactingExternalForcePointTwo = allContactingExternalForcePoints.get(indexOfContactingPair);

         if (index == indexOfContactingPair)
         {
            throw new RuntimeException();
         }

         if (allContactingExternalForcePoints.get(contactingExternalForcePointTwo.getIndexOfContactingPair()) != contactingExternalForcePointOne)
         {
            throw new RuntimeException();
         }

         if (performSpringDamper)
         {
            if (index < indexOfContactingPair)
               performSpringDamper(contactingExternalForcePointOne, contactingExternalForcePointTwo);
         }

         if (slipTowardEachOtherIfSlipping)
         {
            if (index < indexOfContactingPair)
               slipTowardEachOtherIfSlipping(contactingExternalForcePointOne, contactingExternalForcePointTwo);
         }
      }

   }

   @Override
   public void addListener(CollisionHandlerListener listener)
   {
      listeners.add(listener);
   }

   @Override
   public void handle(Contacts contacts)
   {
      shapesInContactList.add(contacts);
   }

   @Override
   public void handleCollisions(CollisionDetectionResult results)
   {
      {
         this.maintenanceBeforeCollisionDetection();

         for (int i = 0; i < results.getNumberOfCollisions(); i++)
         {
            Contacts collision = results.getCollision(i);
            handle(collision);
         }

         this.maintenanceAfterCollisionDetection();
      }
   }

   @Override
   public void addContactingExternalForcePoints(Link link, ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      int index = allContactingExternalForcePoints.size();

      for (int i = 0; i < contactingExternalForcePoints.size(); i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
         contactingExternalForcePoint.setIndex(index);
         allContactingExternalForcePoints.add(contactingExternalForcePoint);
         index++;
      }
   }

   private final ArrayList<Integer> indices = new ArrayList<Integer>();
   private double velocityForMicrocollision = 0.01;
   private int numberOfCyclesPerContactPair = 1;
   private double minDistanceToConsiderDifferent = 0.003;
   private double percentMoveTowardTouchdownWhenSamePoint = 0.2;

   private double maximumPenetrationToStart = 0.002;

   private final Point3D point1 = new Point3D();
   private final Point3D point2 = new Point3D();

   private final Point3D tempPoint = new Point3D();
   private final Vector3D tempVectorForAveraging = new Vector3D();

   private final Vector3D normal = new Vector3D();
   private final Vector3D negative_normal = new Vector3D();

   private final Point3D positionOne = new Point3D();
   private final Point3D positionTwo = new Point3D();
   private final Vector3D slipVector = new Vector3D();
   private final Vector3D tempNormal = new Vector3D();

   private void handleLocal(CollisionShapeWithLink shape, Contacts contacts)
   {
      //      Link link = shape.getLink();
      //
      //      int numberOfContacts = contacts.getNumberOfContacts();
      //      indices.clear();
      //
      //      for (int i = 0; i < numberOfContacts; i++)
      //      {
      //         indices.add(i);
      //      }
      //
      //      for (int cycle = 0; cycle < numberOfCyclesPerContactPair; cycle++)
      //      {
      //         Collections.shuffle(indices, random);
      //         if (numberOfContacts > 1)
      //         {
      //            throw new RuntimeException("Only expecting one deepest contact each time...");
      //         }
      //
      //         for (int j = 0; j < numberOfContacts; j++)
      //         {
      //            int i = indices.get(j);
      //            double distance = contacts.getDistance(i);
      //
      //            if (distance > 0.0)
      //               continue;
      //
      //            contacts.getWorldA(i, point1);
      //            contacts.getWorldB(i, point2);
      //            contacts.getWorldNormal(i, normal);
      //
      //            if (!contacts.isNormalOnA())
      //            {
      //               normal.scale(-1.0);
      //            }
      //
      //            if (Double.isNaN(normal.getX()))
      //               throw new RuntimeException("Normal is invalid. Contains NaN!");
      //
      //            negative_normal.set(normal);
      //            negative_normal.scale(-1.0);
      //
      //            ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsOne = link.getContactingExternalForcePoints();
      //
      //            ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeOne = getPointsThatAreContactingGround(contactingExternalForcePointsOne);
      //
      //            int pointsThatAreHoldingWeight = pointsThatAreContactingShapeOne.size();
      //            if (pointsThatAreHoldingWeight > 3)
      //               pointsThatAreHoldingWeight = 3;
      //            for (int k = 0; k < pointsThatAreContactingShapeOne.size(); k++)
      //            {
      //               pointsThatAreContactingShapeOne.get(k).setNumberOfPointsInContactWithSameShape(pointsThatAreHoldingWeight);
      //            }
      //
      //            ContactingExternalForcePoint externalForcePointOne = null;
      //
      //            setSurfaceNormalToMatchNewCollision(pointsThatAreContactingShapeOne, normal);
      //            removeContactOnPointsThatAreOutsideCollisionSandwhich(pointsThatAreContactingShapeOne, point1, normal);
      //            // TODO : is this essential?
      //            //rollContactPointsIfRolling(pointsThatAreContactingShapeTwo);
      //
      //            for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
      //            {
      //               ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());
      //
      //
      //               Vector3D deltaVectorRemovingNormalComponentsTwo = new Vector3D(contactPointToConsiderTwo.getPositionPoint());
      //               deltaVectorRemovingNormalComponentsTwo.sub(point2);
      //               subtractOffNormalComponent(normal, deltaVectorRemovingNormalComponentsTwo);
      //               double distanceToConsiderTwo = deltaVectorRemovingNormalComponentsTwo.length();
      //
      //               if ((distanceToConsiderTwo < minDistanceToConsiderDifferent))
      //               {
      //                  externalForcePointOne = contactPointToConsiderOne;
      //                  externalForcePointTwo = contactPointToConsiderTwo;
      //                  contactPairAlreadyExists = true;
      //
      //                  boolean areSlipping = true;
      //
      //                  if (areSlipping)
      //                  {
      //                     contactPointToConsiderOne.getPosition(positionOne);
      //                     slipVector.set(deltaVectorRemovingNormalComponentsOne);
      //                     slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
      //                     positionOne.sub(slipVector);
      //                     contactPointToConsiderOne.setOffsetWorld(positionOne);
      //
      //                     contactPointToConsiderTwo.getPosition(positionTwo);
      //                     slipVector.set(deltaVectorRemovingNormalComponentsTwo);
      //                     slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
      //                     positionTwo.sub(slipVector);
      //                     contactPointToConsiderTwo.setOffsetWorld(positionTwo);
      //                  }
      //
      //                  break;
      //               }
      //            }
      //
      //            if (!contactPairAlreadyExists)
      //            {
      //               externalForcePointOne = getAvailableContactingExternalForcePoint(contactingExternalForcePointsOne);
      //               externalForcePointTwo = getAvailableContactingExternalForcePoint(contactingExternalForcePointsTwo);
      //
      //               if ((externalForcePointOne != null) && (externalForcePointTwo != null))
      //               {
      //                  if (createNewContactPairs)
      //                  {
      //                     externalForcePointOne.setIndexOfContactingPair(externalForcePointTwo.getIndex());
      //                     externalForcePointTwo.setIndexOfContactingPair(externalForcePointOne.getIndex());
      //
      //                     externalForcePointOne.setCollisionShape(shape1);
      //                     externalForcePointTwo.setCollisionShape(shape2);
      //                  }
      //               }
      //               else
      //               {
      //                  throw new RuntimeException("No more contact pairs are available!");
      //               }
      //            }
      //
      //            int indexOfOne = externalForcePointOne.getIndex();
      //            int indexOfTwo = externalForcePointTwo.getIndex();
      //
      //            int indexOfContactingPairOne = externalForcePointOne.getIndexOfContactingPair();
      //            int indexOfContactingPairTwo = externalForcePointTwo.getIndexOfContactingPair();
      //
      //            if (createNewContactPairs)
      //            {
      //               if (indexOfOne != indexOfContactingPairTwo)
      //               {
      //                  throw new RuntimeException("");
      //               }
      //
      //               if (indexOfTwo != indexOfContactingPairOne)
      //               {
      //                  throw new RuntimeException("");
      //               }
      //
      //               if (allContactingExternalForcePoints.get(indexOfOne) != externalForcePointOne)
      //               {
      //                  throw new RuntimeException("Contacting pair indices are not consistent!!!");
      //               }
      //
      //               if (allContactingExternalForcePoints.get(indexOfTwo) != externalForcePointTwo)
      //               {
      //                  throw new RuntimeException("Contacting pair indices are not consistent!!!");
      //               }
      //            }
      //
      //            if (!contactPairAlreadyExists)
      //            {
      //               externalForcePointOne.setSurfaceNormalInWorld(normal);
      //               externalForcePointTwo.setSurfaceNormalInWorld(negative_normal);
      //
      //               if (useAverageNewCollisionTouchdownPoints)
      //               {
      //                  tempVectorForAveraging.set(point2);
      //                  tempVectorForAveraging.sub(point1);
      //                  tempVectorForAveraging.scale(0.5);
      //
      //                  double penetrationLength = tempVectorForAveraging.length();
      //                  if (penetrationLength > maximumPenetrationToStart)
      //                  {
      //                     tempVectorForAveraging.scale(maximumPenetrationToStart / penetrationLength);
      //                  }
      //
      //                  tempPoint.set(point1);
      //                  tempPoint.add(tempVectorForAveraging);
      //                  externalForcePointOne.setOffsetWorld(tempPoint);
      //
      //                  tempPoint.set(point2);
      //                  tempPoint.sub(tempVectorForAveraging);
      //                  externalForcePointTwo.setOffsetWorld(tempPoint);
      //               }
      //               else
      //               {
      //                  externalForcePointOne.setOffsetWorld(point1);
      //                  externalForcePointTwo.setOffsetWorld(point2);
      //               }
      //            }
      //
      //            Robot robot1 = linkOne.getParentJoint().getRobot();
      //            Robot robot2 = linkTwo.getParentJoint().getRobot();
      //
      //            robot1.update();
      //            robot1.updateVelocities();
      //
      //            if (robot2 != robot1)
      //            {
      //               robot2.update();
      //               robot2.updateVelocities();
      //            }
      //
      //            //            if (resolveCollisionWithAnImpact && (!contactPairAlreadyExists || !performSpringDamper))
      //            //               resolveCollisionWithAnImpact(shape1, shape2, shapeOneIsGround, shapeTwoIsGround, externalForcePointOne, externalForcePointTwo,
      //            //                                            allowMicroCollisions);
      //
      //         }
      //      }
   }

   
   
   private void handleLocal(CollisionShapeWithLink shape1, CollisionShapeWithLink shape2, Contacts contacts)
   {
      boolean shapeOneIsGround = shape1.isGround();
      boolean shapeTwoIsGround = shape2.isGround();


      if (shapeOneIsGround && shapeTwoIsGround)
      {
         return;
      }
      
      Link linkOne = shape1.getLink();
      Link linkTwo = shape2.getLink();

      int numberOfContacts = contacts.getNumberOfContacts();
      indices.clear();

      for (int i = 0; i < numberOfContacts; i++)
      {
         indices.add(i);
      }

      for (int cycle = 0; cycle < numberOfCyclesPerContactPair; cycle++)
      {
         Collections.shuffle(indices, random);
         if (numberOfContacts > 1)
         {
            throw new RuntimeException("Only expecting one deepest contact each time...");
         }

         for (int j = 0; j < numberOfContacts; j++)
         {
            int i = indices.get(j);
            double distance = contacts.getDistance(i);

            if (distance > 0.0)
               continue;

            contacts.getWorldA(i, point1);
            contacts.getWorldB(i, point2);

            contacts.getWorldNormal(i, normal);

            if (!contacts.isNormalOnA())
            {
               normal.scale(-1.0);
            }

            if (Double.isNaN(normal.getX()))
               throw new RuntimeException("Normal is invalid. Contains NaN!");

            negative_normal.set(normal);
            negative_normal.scale(-1.0);

            ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsOne = linkOne.getContactingExternalForcePoints();
            ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsTwo = linkTwo.getContactingExternalForcePoints();

            boolean contactPairAlreadyExists = false;
            ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeOne = getPointsThatAreContactingOtherLink(contactingExternalForcePointsTwo,
                                                                                                                          linkOne);
            ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo = getPointsThatAreContactingOtherLink(contactingExternalForcePointsOne,
                                                                                                                          linkTwo);

            int pointsThatAreHoldingWeight = pointsThatAreContactingShapeOne.size();
            if (pointsThatAreHoldingWeight > 3)
               pointsThatAreHoldingWeight = 3;
            for (int k = 0; k < pointsThatAreContactingShapeOne.size(); k++)
            {
               pointsThatAreContactingShapeOne.get(k).setNumberOfPointsInContactWithSameShape(pointsThatAreHoldingWeight);
            }

            for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
            {
               pointsThatAreContactingShapeTwo.get(k).setNumberOfPointsInContactWithSameShape(pointsThatAreContactingShapeTwo.size());
            }

            ContactingExternalForcePoint externalForcePointOne = null;
            ContactingExternalForcePoint externalForcePointTwo = null;

            setSurfaceNormalToMatchNewCollision(pointsThatAreContactingShapeTwo, normal, negative_normal);
            removeContactOnPointsThatAreOutsideCollisionSandwhich(pointsThatAreContactingShapeTwo, point1, normal, point2, negative_normal);
            rollContactPointsIfRolling(pointsThatAreContactingShapeTwo);

            // Pick the existing pair that is close enough to the contacts:
            for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
            {
               ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
               ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

               Vector3D deltaVectorRemovingNormalComponentsOne = new Vector3D(contactPointToConsiderOne.getPositionPoint());
               deltaVectorRemovingNormalComponentsOne.sub(point1);
               subtractOffNormalComponent(normal, deltaVectorRemovingNormalComponentsOne);
               double distanceToConsiderOne = deltaVectorRemovingNormalComponentsOne.length();

               Vector3D deltaVectorRemovingNormalComponentsTwo = new Vector3D(contactPointToConsiderTwo.getPositionPoint());
               deltaVectorRemovingNormalComponentsTwo.sub(point2);
               subtractOffNormalComponent(normal, deltaVectorRemovingNormalComponentsTwo);
               double distanceToConsiderTwo = deltaVectorRemovingNormalComponentsTwo.length();

               if ((distanceToConsiderOne < minDistanceToConsiderDifferent) || (distanceToConsiderTwo < minDistanceToConsiderDifferent))
               {
                  externalForcePointOne = contactPointToConsiderOne;
                  externalForcePointTwo = contactPointToConsiderTwo;
                  contactPairAlreadyExists = true;

                  boolean areSlipping = true;

                  if (areSlipping)
                  {
                     contactPointToConsiderOne.getPosition(positionOne);
                     slipVector.set(deltaVectorRemovingNormalComponentsOne);
                     slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
                     positionOne.sub(slipVector);
                     contactPointToConsiderOne.setOffsetWorld(positionOne);

                     contactPointToConsiderTwo.getPosition(positionTwo);
                     slipVector.set(deltaVectorRemovingNormalComponentsTwo);
                     slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
                     positionTwo.sub(slipVector);
                     contactPointToConsiderTwo.setOffsetWorld(positionTwo);
                  }

                  break;
               }
            }

            if (!contactPairAlreadyExists)
            {
               externalForcePointOne = getAvailableContactingExternalForcePoint(contactingExternalForcePointsOne);
               externalForcePointTwo = getAvailableContactingExternalForcePoint(contactingExternalForcePointsTwo);

               if ((externalForcePointOne != null) && (externalForcePointTwo != null))
               {
                  if (createNewContactPairs)
                  {
                     externalForcePointOne.setIndexOfContactingPair(externalForcePointTwo.getIndex());
                     externalForcePointTwo.setIndexOfContactingPair(externalForcePointOne.getIndex());

                     externalForcePointOne.setCollisionShape(shape1);
                     externalForcePointTwo.setCollisionShape(shape2);
                  }
               }
               else
               {
                  throw new RuntimeException("No more contact pairs are available!");
               }
            }

            int indexOfOne = externalForcePointOne.getIndex();
            int indexOfTwo = externalForcePointTwo.getIndex();

            int indexOfContactingPairOne = externalForcePointOne.getIndexOfContactingPair();
            int indexOfContactingPairTwo = externalForcePointTwo.getIndexOfContactingPair();

            if (createNewContactPairs)
            {
               if (indexOfOne != indexOfContactingPairTwo)
               {
                  throw new RuntimeException("");
               }

               if (indexOfTwo != indexOfContactingPairOne)
               {
                  throw new RuntimeException("");
               }

               if (allContactingExternalForcePoints.get(indexOfOne) != externalForcePointOne)
               {
                  throw new RuntimeException("Contacting pair indices are not consistent!!!");
               }

               if (allContactingExternalForcePoints.get(indexOfTwo) != externalForcePointTwo)
               {
                  throw new RuntimeException("Contacting pair indices are not consistent!!!");
               }
            }

            if (!contactPairAlreadyExists)
            {
               externalForcePointOne.setSurfaceNormalInWorld(normal);
               externalForcePointTwo.setSurfaceNormalInWorld(negative_normal);

               if (useAverageNewCollisionTouchdownPoints)
               {
                  tempVectorForAveraging.set(point2);
                  tempVectorForAveraging.sub(point1);
                  tempVectorForAveraging.scale(0.5);

                  double penetrationLength = tempVectorForAveraging.length();
                  if (penetrationLength > maximumPenetrationToStart)
                  {
                     tempVectorForAveraging.scale(maximumPenetrationToStart / penetrationLength);
                  }

                  tempPoint.set(point1);
                  tempPoint.add(tempVectorForAveraging);
                  externalForcePointOne.setOffsetWorld(tempPoint);

                  tempPoint.set(point2);
                  tempPoint.sub(tempVectorForAveraging);
                  externalForcePointTwo.setOffsetWorld(tempPoint);
               }
               else
               {
                  externalForcePointOne.setOffsetWorld(point1);
                  externalForcePointTwo.setOffsetWorld(point2);
               }
            }

            Robot robot1 = linkOne.getParentJoint().getRobot();
            Robot robot2 = linkTwo.getParentJoint().getRobot();

            robot1.update();
            robot1.updateVelocities();

            if (robot2 != robot1)
            {
               robot2.update();
               robot2.updateVelocities();
            }

            //            if (resolveCollisionWithAnImpact && (!contactPairAlreadyExists || !performSpringDamper))
            //               resolveCollisionWithAnImpact(shape1, shape2, shapeOneIsGround, shapeTwoIsGround, externalForcePointOne, externalForcePointTwo,
            //                                            allowMicroCollisions);

         }

      }

   }

   private void performSpringDamper(ContactingExternalForcePoint contactingExternalForcePointOne, ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      Point3D position = new Point3D();
      Vector3D velocity = new Vector3D();
      Vector3D angularVelocity = new Vector3D();
      Vector3D normal = new Vector3D();

      Point3D matchingPosition = new Point3D();
      Vector3D matchingVelocity = new Vector3D();
      Vector3D matchingAngularVelocity = new Vector3D();
      Vector3D matchingNormal = new Vector3D();

      contactingExternalForcePointOne.getPosition(position);
      contactingExternalForcePointOne.getVelocity(velocity);
      contactingExternalForcePointOne.getAngularVelocity(angularVelocity);
      contactingExternalForcePointOne.getSurfaceNormalInWorld(normal);

      contactingExternalForcePointTwo.getPosition(matchingPosition);
      contactingExternalForcePointTwo.getVelocity(matchingVelocity);
      contactingExternalForcePointTwo.getAngularVelocity(matchingAngularVelocity);
      contactingExternalForcePointTwo.getSurfaceNormalInWorld(matchingNormal);

      Vector3D positionDifference = new Vector3D();
      Vector3D velocityDifference = new Vector3D();
      Vector3D angularVelocityDifference = new Vector3D();

      positionDifference.set(matchingPosition);
      positionDifference.sub(position);

      velocityDifference.set(matchingVelocity);
      velocityDifference.sub(velocity);

      angularVelocityDifference.set(matchingAngularVelocity);
      angularVelocityDifference.sub(angularVelocity);

      boolean pullingOut = false;
      if (velocityDifference.dot(normal) > 0.005)
      {
         pullingOut = true;
      }

      Vector3D springForce = new Vector3D();
      Vector3D damperForce = new Vector3D();
      Vector3D rotationalDamperMoment = new Vector3D();

      springForce.set(positionDifference);
      springForce.scale(kpCollision);
      if (pullingOut)
      {
         springForce.scale(pullingOutSpringHysteresisReduction);
      }

      damperForce.set(velocityDifference);
      damperForce.scale(kdCollision);

      rotationalDamperMoment.set(angularVelocityDifference);
      rotationalDamperMoment.scale(kdRotationalDamping);

      Vector3D totalForce = new Vector3D();
      totalForce.set(springForce);
      totalForce.add(damperForce);

      double numberOfPointsContacting = (double) contactingExternalForcePointOne.getNumberOfPointsInContactWithSameShape();
      if (numberOfPointsContacting < 1.0)
         numberOfPointsContacting = 1.0;

      if (divideByNumberContacting)
      {
         totalForce.scale(1.0 / numberOfPointsContacting);
      }

      Vector3D forceAlongNormal = new Vector3D(normal);
      forceAlongNormal.scale(totalForce.dot(normal) / (normal.dot(normal)));

      Vector3D forcePerpendicularToNormal = new Vector3D(totalForce);
      forcePerpendicularToNormal.sub(forceAlongNormal);

      double momentToForceRatio = rotationalDamperMoment.length() / forceAlongNormal.length();

      if (momentToForceRatio > rotationalCoefficientOfFrictionBeta)
      {
         rotationalDamperMoment.scale(rotationalCoefficientOfFrictionBeta / momentToForceRatio);
      }

      double forceRatio = forcePerpendicularToNormal.length() / forceAlongNormal.length();

      if (forceAlongNormal.dot(normal) >= 0.0)
      {
         contactingExternalForcePointOne.setForce(0.0, 0.0, 0.0);
         contactingExternalForcePointTwo.setForce(0.0, 0.0, 0.0);
         contactingExternalForcePointOne.setMoment(0.0, 0.0, 0.0);
         contactingExternalForcePointOne.setMoment(0.0, 0.0, 0.0);

         contactingExternalForcePointOne.setIsSlipping(true);
         contactingExternalForcePointTwo.setIsSlipping(true);
      }

      else if (forceRatio > coefficientOfFriction)
      {
         forcePerpendicularToNormal.scale(coefficientOfFriction / forceRatio);
         totalForce.set(forceAlongNormal);
         totalForce.add(forcePerpendicularToNormal);

         contactingExternalForcePointOne.setForce(totalForce);
         contactingExternalForcePointOne.setMoment(rotationalDamperMoment);
         totalForce.negate();
         rotationalDamperMoment.negate();
         contactingExternalForcePointTwo.setForce(totalForce);
         contactingExternalForcePointTwo.setMoment(rotationalDamperMoment);

         contactingExternalForcePointOne.setIsSlipping(true);
         contactingExternalForcePointTwo.setIsSlipping(true);
      }

      else
      {
         contactingExternalForcePointOne.setForce(totalForce);
         contactingExternalForcePointOne.setMoment(rotationalDamperMoment);
         totalForce.negate();
         rotationalDamperMoment.negate();
         contactingExternalForcePointTwo.setForce(totalForce);
         contactingExternalForcePointTwo.setMoment(rotationalDamperMoment);

         contactingExternalForcePointOne.setIsSlipping(false);
         contactingExternalForcePointTwo.setIsSlipping(false);
      }

      contactingExternalForcePointOne.setImpulse(0.0, 0.0, 0.0);
      contactingExternalForcePointTwo.setImpulse(0.0, 0.0, 0.0);
   }

   private void slipTowardEachOtherIfSlipping(ContactingExternalForcePoint contactingExternalForcePointOne,
                                              ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      boolean areSlipping = areSlipping(contactingExternalForcePointOne, contactingExternalForcePointTwo);

      if (areSlipping)
      {
         CollisionShapeWithLink collisionShapeOne = contactingExternalForcePointOne.getCollisionShape();
         CollisionShapeWithLink collisionShapeTwo = contactingExternalForcePointTwo.getCollisionShape();

         contactingExternalForcePointOne.getPosition(positionOne);
         contactingExternalForcePointTwo.getPosition(positionTwo);

         boolean isPointOneInside = collisionShapeTwo.getTransformedCollisionShapeDescription().isPointInside(positionOne);
         boolean isPointTwoInside = collisionShapeOne.getTransformedCollisionShapeDescription().isPointInside(positionTwo);

         if ((!isPointOneInside) && (!isPointTwoInside))
         {
            contactingExternalForcePointOne.setIndexOfContactingPair(-1);
            contactingExternalForcePointTwo.setIndexOfContactingPair(-1);
            return;
         }

         slipVector.set(positionTwo);
         slipVector.sub(positionOne);

         contactingExternalForcePointOne.getSurfaceNormalInWorld(tempNormal);
         subtractOffNormalComponent(tempNormal, slipVector);
         slipVector.scale(0.05);

         positionOne.add(slipVector);
         positionTwo.sub(slipVector);

         if (isPointOneInside)
         {
            contactingExternalForcePointTwo.setOffsetWorld(positionTwo);
         }

         if (isPointTwoInside)
         {
            contactingExternalForcePointOne.setOffsetWorld(positionOne);
         }
      }
   }

   private boolean areSlipping(ContactingExternalForcePoint contactingExternalForcePointOne, ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      boolean isSlippingOne = contactingExternalForcePointOne.getIsSlipping();
      boolean isSlippingTwo = contactingExternalForcePointOne.getIsSlipping();

      if (isSlippingOne != isSlippingTwo)
      {
         throw new RuntimeException("Inconsistent isSlipping states!?");
      }

      return isSlippingOne;
   }

   private ArrayList<ContactingExternalForcePoint> getPointsThatAreContactingGround(ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      ArrayList<ContactingExternalForcePoint> pointsThatAreContactingGround = new ArrayList<>();

      for (int k = 0; k < contactingExternalForcePoints.size(); k++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = contactingExternalForcePoints.get(k);
         int indexOfContactingPair = contactingExternalForcePointOne.getIndexOfContactingPair();

         if (indexOfContactingPair == -1)
         {
            pointsThatAreContactingGround.add(contactingExternalForcePointOne);
         }
      }

      return pointsThatAreContactingGround;
   }

   private ArrayList<ContactingExternalForcePoint> getPointsThatAreContactingOtherLink(ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsOne,
                                                                                       Link linkTwo)
   {
      ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo = new ArrayList<>();

      for (int k = 0; k < contactingExternalForcePointsOne.size(); k++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = contactingExternalForcePointsOne.get(k);
         int indexOfContactingPair = contactingExternalForcePointOne.getIndexOfContactingPair();

         if (indexOfContactingPair != -1)
         {
            ContactingExternalForcePoint brotherContactingExternalForcePointTwo = allContactingExternalForcePoints.get(indexOfContactingPair);
            if (brotherContactingExternalForcePointTwo.getLink() == linkTwo)
            {
               pointsThatAreContactingShapeTwo.add(contactingExternalForcePointOne);
            }
         }
      }
      return pointsThatAreContactingShapeTwo;
   }

   private final Vector3D tempForceVector = new Vector3D();

   private ContactingExternalForcePoint getAvailableContactingExternalForcePoint(ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      for (int i = 0; i < contactingExternalForcePoints.size(); i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
         if (contactingExternalForcePoint.getIndexOfContactingPair() == -1)
         {
            return contactingExternalForcePoint;
         }
      }

      if (allowRecyclingOfPointsInUse)
      {
         int indexWithSmallestForce = -1;
         double smallestForceSquared = Double.POSITIVE_INFINITY;

         for (int i = 0; i < contactingExternalForcePoints.size(); i++)
         {
            contactingExternalForcePoints.get(i).getForce(tempForceVector);
            double forceSquared = tempForceVector.dot(tempForceVector);
            if (forceSquared < smallestForceSquared)
            {
               smallestForceSquared = forceSquared;
               indexWithSmallestForce = i;
            }
         }

         ContactingExternalForcePoint contactingExternalForcePointToRecycleOne = contactingExternalForcePoints.get(indexWithSmallestForce);
         int indexOfContactingPair = contactingExternalForcePointToRecycleOne.getIndexOfContactingPair();
         ContactingExternalForcePoint contactingExternalForcePointToRecycleTwo = allContactingExternalForcePoints.get(indexOfContactingPair);

         contactingExternalForcePointToRecycleOne.setIndexOfContactingPair(-1);
         contactingExternalForcePointToRecycleTwo.setIndexOfContactingPair(-1);

         return contactingExternalForcePointToRecycleOne;
      }
      else
      {
         System.err.println("No more contact pairs are available!");
         System.err.println("contactingExternalForcePoints.size() = " + contactingExternalForcePoints.size());

         for (int i = 0; i < contactingExternalForcePoints.size(); i++)
         {
            ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
            System.err.println("contactingExternalForcePoint = " + contactingExternalForcePoint.getPositionPoint());
         }
      }

      PrintTools.info("null");
      return null;
   }

   private void setSurfaceNormalToMatchNewCollision(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo, Vector3D normal,
                                                    Vector3D negativeNormal)
   {
      for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.setSurfaceNormalInWorld(normal);
         contactPointToConsiderTwo.setSurfaceNormalInWorld(negativeNormal);
      }
   }

   private void setSurfaceNormalToMatchNewCollision(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShape, Vector3D normal)
   {
      for (int k = 0; k < pointsThatAreContactingShape.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsider = pointsThatAreContactingShape.get(k);
         contactPointToConsider.setSurfaceNormalInWorld(normal);
      }
   }

   private final ArrayList<ContactingExternalForcePoint> pointsToRemove = new ArrayList<>();
   private final Point3D positionOneToConsider = new Point3D();
   private final Point3D positionTwoToConsider = new Point3D();
   private final Vector3D tempVector = new Vector3D();

   private void removeContactOnPointsThatAreOutsideCollisionSandwhich(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo, Point3D point1,
                                                                      Vector3D normal, Point3D point2, Vector3D negativeNormal)
   {
      pointsToRemove.clear();

      for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.getPosition(positionOneToConsider);
         contactPointToConsiderTwo.getPosition(positionTwoToConsider);

         tempVector.set(positionTwoToConsider);
         tempVector.sub(point1);
         if (tempVector.dot(normal) > 0.0)
         {
            contactPointToConsiderOne.setIndexOfContactingPair(-1);
            contactPointToConsiderTwo.setIndexOfContactingPair(-1);

            pointsToRemove.add(contactPointToConsiderOne);
         }
         else
         {
            tempVector.set(positionOneToConsider);
            tempVector.sub(point2);
            if (tempVector.dot(negativeNormal) > 0.0)
            {
               contactPointToConsiderOne.setIndexOfContactingPair(-1);
               contactPointToConsiderTwo.setIndexOfContactingPair(-1);

               pointsToRemove.add(contactPointToConsiderOne);
            }
         }
      }

      pointsThatAreContactingShapeTwo.removeAll(pointsToRemove);
   }

   private void removeContactOnPointsThatAreOutsideCollisionSandwhich(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShape, Point3D point1,
                                                                      Vector3D normal)
   {
      pointsToRemove.clear();

      for (int k = 0; k < pointsThatAreContactingShape.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsider = pointsThatAreContactingShape.get(k);

         contactPointToConsider.getPosition(positionOneToConsider);

         tempVector.set(positionOneToConsider);
         tempVector.sub(point1);
         if (tempVector.dot(normal) > 0.0)
         {
            contactPointToConsider.setIndexOfContactingPair(-1);

            pointsToRemove.add(contactPointToConsider);
         }
         else
         {

         }
      }

      pointsThatAreContactingShape.removeAll(pointsToRemove);
   }

   private final Point3D tempPositionForRollingOne = new Point3D();
   private final Vector3D tempSurfaceNormalForRolllingOne = new Vector3D();
   private final Point3D tempPositionForRollingTwo = new Point3D();
   private final Vector3D tempSurfaceNormalForRolllingTwo = new Vector3D();
   private final Vector3D tempVectorForRolling = new Vector3D();

   private void rollContactPointsIfRolling(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo)
   {
      for (int k = 0; k < pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.getPosition(tempPositionForRollingOne);
         contactPointToConsiderOne.getSurfaceNormalInWorld(tempSurfaceNormalForRolllingOne);
         CollisionShapeWithLink collisionShapeOne = contactPointToConsiderOne.getCollisionShape();
         CollisionShapeDescription<?> collisionShapeDescriptionOne = collisionShapeOne.getTransformedCollisionShapeDescription();
         boolean wasRollingOne = collisionShapeDescriptionOne.rollContactIfRolling(tempSurfaceNormalForRolllingOne, tempPositionForRollingOne);
         contactPointToConsiderOne.setOffsetWorld(tempPositionForRollingOne);

         contactPointToConsiderTwo.getPosition(tempPositionForRollingTwo);
         contactPointToConsiderTwo.getSurfaceNormalInWorld(tempSurfaceNormalForRolllingTwo);
         CollisionShapeWithLink collisionShapeTwo = contactPointToConsiderTwo.getCollisionShape();
         CollisionShapeDescription<?> collisionShapeDescriptionTwo = collisionShapeTwo.getTransformedCollisionShapeDescription();
         boolean wasRollingTwo = collisionShapeDescriptionTwo.rollContactIfRolling(tempSurfaceNormalForRolllingTwo, tempPositionForRollingTwo);
         contactPointToConsiderTwo.setOffsetWorld(tempPositionForRollingTwo);

         if (wasRollingOne && wasRollingTwo)
         {
            return;
         }

         if (!wasRollingOne && !wasRollingTwo)
         {
            return;
         }

         if (wasRollingOne)
         {
            tempVectorForRolling.set(tempPositionForRollingOne);
            tempVectorForRolling.sub(tempPositionForRollingTwo);
            subtractOffNormalComponent(tempSurfaceNormalForRolllingOne, tempVectorForRolling);

            tempPositionForRollingTwo.add(tempVectorForRolling);
            contactPointToConsiderTwo.setOffsetWorld(tempPositionForRollingTwo);
         }

         if (wasRollingTwo)
         {
            tempVectorForRolling.set(tempPositionForRollingTwo);
            tempVectorForRolling.sub(tempPositionForRollingOne);
            subtractOffNormalComponent(tempSurfaceNormalForRolllingTwo, tempVectorForRolling);

            tempPositionForRollingOne.add(tempVectorForRolling);
            contactPointToConsiderOne.setOffsetWorld(tempPositionForRollingOne);
         }
      }
   }

   private final Vector3D normalComponent = new Vector3D();

   private Vector3D subtractOffNormalComponent(Vector3D normal, Vector3D vectorToRemoveNormalComponent)
   {
      double percentOfNormalComponent = vectorToRemoveNormalComponent.dot(normal) / (normal.dot(normal));
      normalComponent.set(normal);
      normalComponent.scale(percentOfNormalComponent);
      vectorToRemoveNormalComponent.sub(normalComponent);
      return vectorToRemoveNormalComponent;
   }
}
