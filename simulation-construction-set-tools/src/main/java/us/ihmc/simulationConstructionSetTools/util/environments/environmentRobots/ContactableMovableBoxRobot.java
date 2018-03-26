package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.geometry.shapes.FrameBox3d;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObject;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.inputDevices.keyboard.Key;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

import java.awt.*;
import java.util.ArrayList;

public class ContactableMovableBoxRobot extends ContactableRobot implements TerrainObject3D, HeightMapWithNormals
{
   private static final double DEFAULT_LENGTH = 1.0;
   private static final double DEFAULT_WIDTH = 0.6;
   private static final double DEFAULT_HEIGHT = 1.2;

   private static final double DEFAULT_MASS = 10.0;

   private final FrameBox3d frameBox;
   private final BoundingBox3D boundingBox3D = new BoundingBox3D();

   private final FloatingJoint floatingJoint;
   private final Link boxLink;
   private final Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);

   private final Point3D intersectionA = new Point3D();
   private final Point3D intersectionB = new Point3D();

   // graphics
   private final Graphics3DInstruction boxGraphics;

   public ContactableMovableBoxRobot(String name, double length, double width, double height, double mass)
   {
      super(name);

      floatingJoint = new FloatingJoint(name + "Base", name, new Vector3D(0.0, 0.0, 0.0), this);
      linkGraphics = new Graphics3DObject();
      linkGraphics.setChangeable(true);
      boxLink = boxLink(linkGraphics, length, width, height, mass);
      floatingJoint.setLink(boxLink);
      this.addRootJoint(floatingJoint);

      frameBox = new FrameBox3d(ReferenceFrame.getWorldFrame(), length, width, height);

      Box3D box = frameBox.getBox3d();
      boxGraphics = linkGraphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ());
      box.getBoundingBox3D(boundingBox3D);
      setUpGroundContactPoints(frameBox);
   }

   public static ContactableMovableBoxRobot createContactableCardboardBoxRobot(String name, double length, double width, double height, double mass)
   {
      ContactableMovableBoxRobot contactableBoxRobot = new ContactableMovableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.createCardboardBoxGraphics(length, width, height);

      return contactableBoxRobot;
   }

   public static ContactableMovableBoxRobot createContactableWoodBoxRobot(String name, double length, double width, double height, double mass)
   {
      ContactableMovableBoxRobot contactableBoxRobot = new ContactableMovableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.createWoodBoxGraphics(length, width, height);

      return contactableBoxRobot;
   }

   public static ContactableMovableBoxRobot createContactable2By4Robot(String name, double length, double width, double height, double mass)
   {
      ContactableMovableBoxRobot contactableBoxRobot = new ContactableMovableBoxRobot(name, length, width, height, mass);
      contactableBoxRobot.create2By4Graphics(length, width, height);

      return contactableBoxRobot;
   }

   public void addYoGraphicForceVectorsToGroundContactPoints(double forceVectorScale, AppearanceDefinition appearance,
                                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicForceVectorsToGroundContactPoints(0, forceVectorScale, appearance, yoGraphicsListRegistry);
   }

   public void addYoGraphicForceVectorsToGroundContactPoints(int groupIdentifier, double forceVectorScale, AppearanceDefinition appearance,
                                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
         return;

      GroundContactPointGroup groundContactPointGroup = floatingJoint.getGroundContactPointGroup(groupIdentifier);
      ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPoints();

      for (GroundContactPoint groundContactPoint : groundContactPoints)
      {
         YoGraphicVector yoGraphicVector = new YoGraphicVector(groundContactPoint.getName(), groundContactPoint.getYoPosition(),
                                                               groundContactPoint.getYoForce(), forceVectorScale, appearance);
         yoGraphicsListRegistry.registerYoGraphic("ContactableSelectableBoxRobot", yoGraphicVector);
      }
   }

   private void setUpGroundContactPoints(FrameBox3d frameBox)
   {
      String name = this.getName();

      for (int i = 0; i < 8; i++)
      {
         Point3D vertex = new Point3D();
         frameBox.getBox3d().getVertex(i, vertex);
         GroundContactPoint groundContactPoint = new GroundContactPoint("gc_" + name + i, new Vector3D(vertex), this.getRobotsYoVariableRegistry());

         floatingJoint.addGroundContactPoint(groundContactPoint);
      }
   }

   @Override
   public FloatingJoint getFloatingJoint()
   {
      return floatingJoint;
   }

   private Link boxLink(Graphics3DObject linkGraphics, double length, double width, double height, double mass)
   {
      Link ret = new Link("box");

      ret.setMass(mass);

      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidBox(length, width, height, mass));
      ret.setComOffset(0.0, 0.0, 0.0);

      //      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      //      linkGraphics.addCube(length, width, height, YoAppearance.EarthTexture(null));
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return frameBox.getBox3d().isInsideOrOnSurface(pointInWorldToCheck);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      return checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }

   private boolean checkIfInside(Point3D pointToCheck, Point3D intersectionToPack, Vector3D normalToPack)
   {
      if (!isPointOnOrInside(pointToCheck))
         return false;

      return frameBox.getBox3d().checkIfInside(pointToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      tempPoint.set(x, y, z);
      return isClose(tempPoint);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      frameBox.getBox3d().checkIfInside(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public void setMass(double mass)
   {
      boxLink.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      boxLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }

   private void createCardboardBoxGraphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/cardboardBox.obj", sizeX, sizeY, sizeZ);
   }

   private void createWoodBoxGraphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/woodBox2.obj", sizeX, sizeY, sizeZ);
   }

   // TODO Create some graphics for the 2-by-4 debris
   private void create2By4Graphics(double sizeX, double sizeY, double sizeZ)
   {
      add1x13DObject("models/woodBox2.obj", sizeX, sizeY, sizeZ);
   }

   protected void add1x13DObject(String fileName, double length, double width, double height)
   {
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.translate(0.0, 0.0, -height / 2.0); // TODO: Center the 3ds files so we don't have to do this translate.
      graphics.scale(new Vector3D(length, width, height));
      graphics.addModelFile(fileName);

      linkGraphics.combine(graphics);
   }

   @Override
   public void update()
   {
      super.update();
      updateCurrentBox3d();
   }

   private final RigidBodyTransform temporaryTransform3D = new RigidBodyTransform();

   private synchronized void updateCurrentBox3d()
   {
      floatingJoint.getTransformToWorld(temporaryTransform3D);
      frameBox.setTransform(temporaryTransform3D);
      frameBox.getBox3d().getBoundingBox3D(boundingBox3D);
   }


   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox3D;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      // TODO: inefficient, magic epsilon
      tempPoint.set(x, y, z);
      zVector.set(0.0, 0.0, 1.0);
      int numberOfIntersections = frameBox.getBox3d().intersectionWith(tempPoint, zVector, intersectionA, intersectionB);

      if (numberOfIntersections == 0)
         return Double.NEGATIVE_INFINITY;
      else if (numberOfIntersections == 1)
         return intersectionA.getZ();
      else
         return Math.max(intersectionA.getZ(), intersectionB.getZ());
   }

   private void surfaceNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      frameBox.checkIfInside(tempPoint, null, normalToPack);
   }

   /**
    * This is the graphics for the terrain object, which is static, and never updated. Instead, we use the graphics that are contained
    * in {@link us.ihmc.simulationconstructionset.Robot}
    * @return
    */
   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return null;
   }
}
