package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Pair;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.shapes.FramePlane3d;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;

/**
 * <p>Title: FrameConvexPolygon2d </p>
 *
 * <p>Description: Describes a planar convex polygon attached to a {@link ReferenceFrame}.
 * The vertices of this polygon are clockwise and are all different.
 * </p>
 *
 * @author IHMC Biped Team
 * @version 1.0
 */
public class FrameConvexPolygon2d extends FrameGeometryObject<FrameConvexPolygon2d, ConvexPolygon2D>
{
   protected final ConvexPolygon2D convexPolygon;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint2D tempPoint2d = new FramePoint2D();

   private final FramePoint2D temporaryCentroid = new FramePoint2D();


   /**
    * Creates an empty convex polygon attached to the world frame.
    */
   public FrameConvexPolygon2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates an empty convex polygon attached to a reference frame.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new ConvexPolygon2D());
      this.convexPolygon = this.getGeometryObject();
      update();
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using a list of Point2d, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, Point2DReadOnly[] vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using an array of Point2d, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param vertices {@code double[N][>=2]} the array of points that is used to creates the vertices. Each row contains one point whereas the (at least) two columns contains the coordinates x and y.
    * The number of vertices of this polygon will be equal to the length of the point array.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, double[][] vertices)
   {
      this(referenceFrame);
      setIncludingFrameAndUpdate(referenceFrame, vertices);
   }

   /**
    * Creates an empty convex polygon attached to the reference frame of the frame vertex, adds N new vertices using a list of {@code List<FramePoint2d>}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(List<FramePoint2D> frameVertices)
   {
      this();
      setIncludingFrameAndUpdate(frameVertices);
   }

   /**
    * Creates an empty convex polygon attached to the reference frame of the frame vertex, adds N new vertices using a list of {@code FrameTuple2dArrayList<FramePoint2d>}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: pointList.size() can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(FrameTuple2dArrayList<FramePoint2D> frameVertices)
   {
      this();
      setIncludingFrameAndUpdate(frameVertices);
   }

   /**
    * Creates an empty convex polygon attached to a reference frame, adds N new vertices using the vertices of another {code ConvexPolygon2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(ReferenceFrame referenceFrame, ConvexPolygon2D otherPolygon)
   {
      super(referenceFrame, new ConvexPolygon2D());
      this.convexPolygon = this.getGeometryObject();

      setIncludingFrameAndUpdate(referenceFrame, otherPolygon);
   }

   /**
    * Creates a polygon with the same properties as the other polygon.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon.
    */
   public FrameConvexPolygon2d(FrameConvexPolygon2d otherPolygon)
   {
      this();
      setIncludingFrameAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the smallest convex hull that contains both polygons.
    * Then it updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    *
    * @param firstPolygon {@code ConvexPolygon2d}
    * @param secondPolygon {@code ConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public FrameConvexPolygon2d(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      this();
      setIncludingFrameAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * Note that calling that method doesn't generate garbage.
    */
   public void clear()
   {
      convexPolygon.clear();
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid, and is attached to a new reference frame.
    * @param referenceFrame {@code ReferenceFrame} reference frame to which this polygon will be attached.
    * Note that calling that method doesn't generate garbage.
    */
   public void clear(ReferenceFrame referenceFrame)
   {
      convexPolygon.clear();
      this.referenceFrame = referenceFrame;
   }

   public void clearAndUpdate(ReferenceFrame referenceFrame)
   {
      convexPolygon.clearAndUpdate();
      this.referenceFrame = referenceFrame;
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    * @throws ReferenceFrameMismatchException
    */
   public void addVertex(FramePoint2D vertex)
   {
      vertex.checkReferenceFrameMatch(referenceFrame);
      convexPolygon.addVertex(vertex);
   }

   public void addVertex(Point2DReadOnly vertex)
   {
      convexPolygon.addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param x {@code double} first coordinate of the new vertex.
    * @param y {@code double} second coordinate of the new vertex.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertex(ReferenceFrame referenceFrame, double x, double y)
   {
      this.referenceFrame.checkReferenceFrameMatch(referenceFrame);
      convexPolygon.addVertex(x, y);
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrame(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexAndChangeFrame(FramePoint2D newVertex)
   {
      tempPoint2d.setIncludingFrame(newVertex);
      tempPoint2d.changeFrame(referenceFrame);
      convexPolygon.addVertex(tempPoint2d);
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex.changeFrameAndProjectToXYPlane(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint2d} the new vertex (it is not modified).
    */
   public void addVertexChangeFrameAndProjectToXYPlane(FramePoint2D newVertex)
   {
      tempPoint2d.setIncludingFrame(newVertex);
      tempPoint2d.changeFrameAndProjectToXYPlane(referenceFrame);
      convexPolygon.addVertex(tempPoint2d);
   }

   /**
    * Add a vertex to this polygon after doing {@code newVertex2d.changeFrameAndProjectToXYPlane(this.referenceFrame)}.
    * Note that this method recycles memory.
    * @param newVertex {@code FramePoint} the new vertex (it is not modified).
    */
   public void addVertexByProjectionOntoXYPlane(FramePoint3D newVertex)
   {
      tempPoint.setIncludingFrame(newVertex);
      tempPoint.changeFrame(referenceFrame);
      addVertex(referenceFrame, tempPoint.getX(), tempPoint.getY());
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d}.
    * Note that this method recycles memory.
    * @param vertices {@code List<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(List<FramePoint2D> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint2D vertex = vertices.get(i);
         addVertex(vertex);
      }
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d}.
    * Note that this method recycles memory.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(FrameTuple2dArrayList<FramePoint2D> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint2D vertex = vertices.get(i);
         addVertex(vertex);
      }
   }

   public void addVertices(FramePoint2D[] vertices)
   {
      for (int i = 0; i < vertices.length; i++)
      {
         FramePoint2D vertex = vertices[i];
         addVertex(vertex);
      }
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code FramePoint2d} after having changed their frame projected them on this polygon frame.
    * Note that this method recycles memory.
    * @param vertices {@code List<FramePoint2d>} the list of new vertices (it is not modified).
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void addVerticesByProjectionOntoXYPlane(List<FramePoint3D> vertices, int numberOfVertices)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         FramePoint3D vertex = vertices.get(i);
         addVertexByProjectionOntoXYPlane(vertex);
      }
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * Note that this method recycles memory.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon that is used to add new vertices to this polygon.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(ConvexPolygon2D otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = otherPolygon.getVertex(i);
         addVertex(vertex);
      }
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * Note that this method recycles memory.
    * @param otherPolygon {@code FrameConvexPolygon2d} the other convex polygon that is used to add new vertices to this polygon.
    * @throws ReferenceFrameMismatchException
    */
   public void addVertices(FrameConvexPolygon2d otherPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(otherPolygon.getReferenceFrame());

      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = otherPolygon.convexPolygon.getVertex(i);
         convexPolygon.addVertex(vertex);
      }
   }

   public void removeVertex(int indexOfVertexToRemove)
   {
      convexPolygon.removeVertex(indexOfVertexToRemove);
   }

   /**
    * Calls {@code ConvexPolygon2d.update()} and updates the {@code FramePoint}s of this {@code FrameConvecPolygon2d}.
    */
   public void update()
   {
      convexPolygon.update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(List<FramePoint2D> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertexByProjectionOntoXYPlane(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    */
   public void setAndUpdate(List<FramePoint3D> vertices)
   {
      clear();
      for (int i = 0; i < vertices.size(); i++)
      {
         addVertexByProjectionOntoXYPlane(vertices.get(i));
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertexByProjectionOntoXYPlane(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code FramePoint[]} the array of points that is used to creates the vertices.
    */
   public void setAndUpdate(FramePoint3D[] vertices)
   {
      clear();
      for (int i = 0; i < vertices.length; i++)
      {
         addVertexByProjectionOntoXYPlane(vertices[i]);
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertex(vertices.get(i)};
    * 3- {@code update()}.
    * @param vertices {@code FramePoint2d[]} the array of points that is used to creates the vertices.
    */
   public void setAndUpdate(FramePoint2D[] vertices)
   {
      clear();
      for (int i = 0; i < vertices.length; i++)
      {
         addVertex(vertices[i]);
      }
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(ConvexPolygon2D otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(FrameConvexPolygon2d otherPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(otherPolygon.getReferenceFrame());
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(firstPolygon)};
    * 2- {@code addVertices(secondPolygon)};
    * 3- {@code update()}.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    * @param firstPolygon {@code FrameConvexPolygon2d}
    * @param secondPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setAndUpdate(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      referenceFrame.checkReferenceFrameMatch(firstPolygon.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(secondPolygon.getReferenceFrame());

      clear();
      addVertices(firstPolygon);
      addVertices(secondPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void setByProjectionOntoXYPlaneAndUpdate(List<FramePoint3D> vertices, int numberOfVertices)
   {
      clear();
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(otherPolygon.getReferenceFrame())};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code FrameConvexPolygon2d}
    */
   public void setIncludingFrameAndUpdate(FrameConvexPolygon2d otherPolygon)
   {
      clear(otherPolygon.getReferenceFrame());
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(firstPolygon.getReferenceFrame())};
    * 2- {@code addVertices(firstPolygon)};
    * 2- {@code addVertices(secondPolygon)};
    * 3- {@code update()}.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    * @param firstPolygon {@code FrameConvexPolygon2d}
    * @param secondPolygon {@code FrameConvexPolygon2d}
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(FrameConvexPolygon2d firstPolygon, FrameConvexPolygon2d secondPolygon)
   {
      referenceFrame = firstPolygon.getReferenceFrame();
      setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(otherPolygon)};
    * 3- {@code update()}.
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param otherPolygon {@code ConvexPolygon2d}
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2D otherPolygon)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.size());
      update();
   }

   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point2DReadOnly[] vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.length);
      update();
   }

   /**
    * If the list of vertices is empty, this polygon will be empty and its reference frame won't be changed.
    * If the list of vertices is not empty, this method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint2d>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(List<FramePoint2D> vertices)
   {
      int numberOfVertices = vertices.size();
      if (numberOfVertices < 1 )
      {
         clear();
         update();
      }
      else
      {
         clear(vertices.get(0).getReferenceFrame());
         addVertices(vertices, numberOfVertices);
         update();
      }
   }

   /**
    * If the list of vertices is empty, this polygon will be empty and its reference frame won't be changed.
    * If the list of vertices is not empty, this method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code FrameTuple2dArrayList<FramePoint2d>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameAndUpdate(FrameTuple2dArrayList<FramePoint2D> vertices)
   {
      int numberOfVertices = vertices.size();
      if (numberOfVertices < 1 )
      {
         clear();
         update();
      }
      else
      {
         clear(vertices.get(0).getReferenceFrame());
         addVertices(vertices, numberOfVertices);
         update();
      }
   }

   /**
    * This method does:
    * 1- {@code clear(referenceFrame)};
    * 2- {@code addVertices(vertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of points that is used to creates the vertices.
    */
   public void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, double[][] vertices)
   {
      clear(referenceFrame);
      this.convexPolygon.addVertices(vertices, vertices.length);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, vertices.size())};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameByProjectionOntoXYPlaneAndUpdate(ReferenceFrame referenceFrame, List<FramePoint3D> vertices)
   {
      int numberOfVertices = vertices.size();
      clear(referenceFrame);
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear(vertices.get(0).getReferenceFrame())};
    * 2- {@code addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param referenceFrame {@code ReferenceFrame} the new reference frame of this polygon.
    * @param vertices {@code List<FramePoint>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int}.
    * @throws ReferenceFrameMismatchException
    */
   public void setIncludingFrameByProjectionOntoXYPlaneAndUpdate(ReferenceFrame referenceFrame, List<FramePoint3D> vertices, int numberOfVertices)
   {
      clear(referenceFrame);
      addVerticesByProjectionOntoXYPlane(vertices, numberOfVertices);
      update();
   }

   public void getVertex(int vertexIndex, Point2DBasics vertexToPack)
   {
      vertexToPack.set(getVertex(vertexIndex));
   }

   public Point2DReadOnly getVertex(int vertexIndex)
   {
      return convexPolygon.getVertex(vertexIndex);
   }

   public FramePoint2D getFrameVertexCopy(int vertexIndex)
   {
      FramePoint2D frameVertexCopy = new FramePoint2D();
      getFrameVertex(vertexIndex, frameVertexCopy);
      return frameVertexCopy;
   }

   public void getFrameVertex(int vertexIndex, FrameTuple2D<?, ?> vertexToPack)
   {
      convexPolygon.checkIfUpToDate();

      convexPolygon.checkNonEmpty();
      convexPolygon.checkIndexInBoundaries(vertexIndex);

      vertexToPack.setIncludingFrame(referenceFrame, convexPolygon.getVertex(vertexIndex));
   }

   public void getFrameVertexXY(int vertexIndex, FramePoint3D vertexToPack)
   {
      convexPolygon.checkIfUpToDate();

      convexPolygon.checkNonEmpty();
      convexPolygon.checkIndexInBoundaries(vertexIndex);

      vertexToPack.setIncludingFrame(referenceFrame, convexPolygon.getVertex(vertexIndex), 0.0);
   }

   public void getNextFrameVertex(int vertexIndex, FramePoint2D vertexToPack)
   {
      getFrameVertex(convexPolygon.getNextVertexIndex(vertexIndex), vertexToPack);
   }

   public void getPreviousFrameVertex(int vertexIndex, FramePoint2D vertexToPack)
   {
      getFrameVertex(convexPolygon.getPreviousVertexIndex(vertexIndex), vertexToPack);
   }

   public double getArea()
   {
      return convexPolygon.getArea();
   }

   public void getCentroid(FramePoint2D centroidToPack)
   {
      centroidToPack.setIncludingFrame(referenceFrame, convexPolygon.getCentroid());
   }

   public FramePoint2D getCentroid()
   {
      getCentroid(temporaryCentroid);
      return temporaryCentroid;
   }

   public FramePoint2D getCentroidCopy()
   {
      FramePoint2D centroidToReturn = new FramePoint2D();
      getCentroid(centroidToReturn);
      return centroidToReturn;
   }

   public int getNumberOfVertices()
   {
      return convexPolygon.getNumberOfVertices();
   }

   public ConvexPolygon2D getConvexPolygon2d()
   {
      convexPolygon.checkIfUpToDate();
      return convexPolygon;
   }

   public ConvexPolygon2D getConvexPolygon2dCopy()
   {
      convexPolygon.checkIfUpToDate();
      return new ConvexPolygon2D(convexPolygon);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * @param pointToScaleAbout
    * @param scaleFactor
    */
   public void scale(Point2DReadOnly pointToScaleAbout, double scaleFactor)
   {
      convexPolygon.scale(pointToScaleAbout, scaleFactor);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * @param pointToScaleAbout
    * @param scaleFactor
    */
   public void scale(FramePoint2D pointToScaleAbout, double scaleFactor)
   {
      checkReferenceFrameMatch(pointToScaleAbout);
      scale(pointToScaleAbout.getPoint(), scaleFactor);
   }

   /**
    * Scale this convex polygon about its centroid, i.e. once scaled the polygon centroid remains unchanged.
    * @param scaleFactor
    */
   public void scale(double scaleFactor)
   {
      convexPolygon.scale(scaleFactor);
   }

   /**
    * Returns distance from the point to the boundary of this polygon. The return value
    * is positive if the point is inside and negative if it is outside.
    */
   public double signedDistance(FramePoint2D point)
   {
      return convexPolygon.signedDistance(point.getPoint());
   }

   public BoundingBox2D getBoundingBoxCopy()
   {
      BoundingBox2D ret = this.convexPolygon.getBoundingBoxCopy();

      return ret;
   }

   public void getBoundingBox(BoundingBox2D boundingBoxToPack)
   {
      this.convexPolygon.getBoundingBox(boundingBoxToPack);
   }

   public boolean isPointInside(FramePoint2D framePoint)
   {
      return isPointInside(framePoint, 0.0);
   }

   /**
    * isPointInside
    * Determines whether a point is inside the convex polygon (point in polygon test).
    * Uses the orientation method (Nordbeck, Rystedt, 1967)
    * Test is only valid for convex polygons, and only if the vertices are ordered clockwise.
    *
    * @param framePoint FramePoint2d the point to be tested
    * @return boolean true if the point is inside the polygon
    */
   public boolean isPointInside(FramePoint2D framePoint, double epsilon)
   {
      framePoint.checkReferenceFrameMatch(referenceFrame);
      return convexPolygon.isPointInside(framePoint, epsilon);
   }

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerFramePoint Point2d
    * @return Point2d[]
    */
   public ArrayList<FramePoint2D> getAllVisibleVerticesFromOutsideLeftToRightCopy(FramePoint2D observerFramePoint)
   {
      this.checkReferenceFrameMatch(observerFramePoint);
      int lineOfSightStartIndex = convexPolygon.lineOfSightStartIndex(observerFramePoint.getPoint());
      int lineOfSightEndIndex = convexPolygon.lineOfSightEndIndex(observerFramePoint.getPoint());
      if (lineOfSightStartIndex == -1 || lineOfSightEndIndex == -1)
         return null;

      ArrayList<FramePoint2D> ret = new ArrayList<FramePoint2D>();
      int index = lineOfSightEndIndex;

      while (true)
      {
         ret.add(getFrameVertexCopy(index));
         index = convexPolygon.getPreviousVertexIndex(index);
         if (index == lineOfSightStartIndex)
         {
            ret.add(getFrameVertexCopy(index));
            break;
         }
      }

      return ret;
   }

   /**
    * Returns the intersecting edges between this polygon and the given FrameLine2d.
    * The FrameLine2d is treated as a ray. This method returns null if:
    * - The start point of the ray starts inside the Polygon,
    * - The ray points away from the polygon.
    * - The intersection is a single vertex,
    * - There is no intersection.
    *
    * @param frameLine2d FrameLine2d Ray to check intersection with this Polygon.
    * @return FrameLineSegment2d[] The two edges that the Ray intersects on this Polygon.
    */
   public FrameLineSegment2d[] getIntersectingEdges(FrameLine2d frameLine2d)
   {
      frameLine2d.checkReferenceFrameMatch(referenceFrame);

      LineSegment2D[] lineSegments = ConvexPolygon2dCalculator.getIntersectingEdgesCopy(frameLine2d.getLine2dCopy(), convexPolygon);
      if (lineSegments == null)
         return null;

      FrameLineSegment2d[] ret = new FrameLineSegment2d[lineSegments.length];

      for (int i = 0; i < lineSegments.length; i++)
      {
         ret[i] = new FrameLineSegment2d(referenceFrame, lineSegments[i]);
      }

      return ret;
   }

   public FramePoint2D getClosestVertexCopy(FramePoint2D point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      return new FramePoint2D(referenceFrame, convexPolygon.getClosestVertexCopy(point.getPoint()));
   }

   public boolean getClosestPointWithRay(FramePoint2D closestVertexToPack, FrameLine2d ray)
   {
      ray.checkReferenceFrameMatch(referenceFrame);
      closestVertexToPack.setToZero(referenceFrame);
      boolean success = convexPolygon.getClosestPointWithRay(ray.getLine2d(), closestVertexToPack);

      return success;
   }

   public void getClosestVertex(FramePoint2D closestVertexToPack, FramePoint2D point)
   {
      point.checkReferenceFrameMatch(referenceFrame);

      closestVertexToPack.setIncludingFrame(referenceFrame, convexPolygon.getClosestVertexCopy(point.getPoint()));
   }

   public FramePoint2D getClosestVertexCopy(FrameLine2d line)
   {
      line.checkReferenceFrameMatch(referenceFrame);

      Point2D closestVertexCopy = convexPolygon.getClosestVertexCopy(line.line);

      if (closestVertexCopy == null)
         throw new RuntimeException("Closest vertex could not be found!");

      return new FramePoint2D(referenceFrame, closestVertexCopy);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      convexPolygon.applyTransformAndProjectToXYPlane(transform);
   }

   public FrameConvexPolygon2d applyTransformCopy(Transform transform)
   {
      FrameConvexPolygon2d copy = new FrameConvexPolygon2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public FrameConvexPolygon2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      FrameConvexPolygon2d copy = new FrameConvexPolygon2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      referenceFrame = desiredFrame;
      applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
   }

   public FrameConvexPolygon2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FrameConvexPolygon2d ret = new FrameConvexPolygon2d(this);
      ret.changeFrameAndProjectToXYPlane(desiredFrame);
      return ret;
   }

   public void orthogonalProjection(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);
      convexPolygon.orthogonalProjection(point);
   }

   public FramePoint2D orthogonalProjectionCopy(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);
      Point2D projected = convexPolygon.orthogonalProjectionCopy(point.getPoint());
      if (projected == null)
      {
         return null;
      }

      return new FramePoint2D(point.getReferenceFrame(), projected);
   }

   public void getNormal3dVector(FrameVector3D normalToPack)
   {
      normalToPack.setIncludingFrame(getReferenceFrame(), 0.0, 0.0, 1.0);
   }
   
   public void getPlane3d(FramePlane3d plane3dToPack)
   {
      plane3dToPack.setIncludingFrame(getReferenceFrame(), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
   }
   
   /**
    * @deprecated Creates garbage. Use an instance of FrameConvexPolygonWithLineIntersector.
    */
   public FramePoint2D[] intersectionWith(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2D[] intersection = this.convexPolygon.intersectionWith(line.line);
      if (intersection == null)
         return null;

      FramePoint2D[] ret = new FramePoint2D[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2D(line.getReferenceFrame(), intersection[i]);
      }

      return ret;
   }
   
   public void intersectionWith(FrameLine2d otherLine, Pair<FramePoint2D, FramePoint2D> intersection)
   {
      checkReferenceFrameMatch(otherLine);
      int numberOfIntersections = convexPolygon.intersectionWith(otherLine.getLine2d(), intersection.getFirst(),
                                                                 intersection.getSecond());
      if (numberOfIntersections < 2)
      {
         intersection.getSecond().setToNaN();
         if (numberOfIntersections < 1)
            intersection.getFirst().setToNaN();
      }
   }

   /**
    * Computes the intersections of a ray with this polygon. Since the polygon is convex the maximum
    * number of intersections is two. Returns the number of intersections found. If there are less
    * then two intersections the FramePoints are set to NaN.
    *
    * @param ray                  ray to intersect this polygon with
    * @param intersectionToPack1  modified - is set to the first intersection
    *                             If the are no intersections this will be set to NaN.
    * @param intersectionToPack2  modified - is set to the second intersection
    *                             If there is only one intersection this will be set to NaN.
    * @return                     The number of intersections 0, 1, or 2
    */
   public int intersectionWithRay(FrameLine2d ray, FramePoint2D intersectionToPack1, FramePoint2D intersectionToPack2)
   {
      checkReferenceFrameMatch(ray);
      intersectionToPack1.setToZero(referenceFrame);
      intersectionToPack2.setToZero(referenceFrame);
      return convexPolygon.intersectionWithRay(ray.getLine2d(), intersectionToPack1, intersectionToPack2);
   }

   public FramePoint2D[] intersectionWithRayCopy(FrameLine2d ray)
   {
      checkReferenceFrameMatch(ray);
      Point2D[] intersections = convexPolygon.intersectionWithRay(ray.getLine2dCopy());
      if (intersections == null)
         return null;

      FramePoint2D[] ret = new FramePoint2D[intersections.length];
      for (int i = 0; i < intersections.length; i++)
      {
         ret[i] = new FramePoint2D(referenceFrame, intersections[i]);
      }

      return ret;
   }

   public FramePoint2D[] intersectionWith(FrameLineSegment2d lineSegment)
   {
      //TODO: Memory inefficient. Don't create new objects...
      checkReferenceFrameMatch(lineSegment);
      Point2D[] intersection = this.convexPolygon.intersectionWith(lineSegment.lineSegment);
      if (intersection == null)
         return null;

      FramePoint2D[] ret = new FramePoint2D[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2D(lineSegment.getReferenceFrame(), intersection[i]);
      }

      return ret;
   }

   public FrameConvexPolygon2d intersectionWith(FrameConvexPolygon2d secondConvexPolygon)
   {
      checkReferenceFrameMatch(secondConvexPolygon);
      ConvexPolygon2D intersection = ConvexPolygonTools.computeIntersectionOfPolygons(this.convexPolygon, secondConvexPolygon.convexPolygon);
      if (intersection == null)
         return null;

      return new FrameConvexPolygon2d(secondConvexPolygon.getReferenceFrame(), intersection);
   }

   public boolean intersectionWith(FrameConvexPolygon2d secondConvexPolygon, FrameConvexPolygon2d intersectionToPack)
   {
      checkReferenceFrameMatch(secondConvexPolygon);
      intersectionToPack.clear(secondConvexPolygon.getReferenceFrame());
      boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon, secondConvexPolygon.convexPolygon, intersectionToPack.convexPolygon);

      return success;
   }

   public double distance(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return this.convexPolygon.distance(point.getPoint());
   }

   public FrameLineSegment2d getClosestEdgeCopy(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return new FrameLineSegment2d(referenceFrame, convexPolygon.getClosestEdgeCopy(point.getPoint()));
   }

   public void getClosestEdge(FrameLineSegment2d closestEdgeToPack, FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      closestEdgeToPack.setToZero(referenceFrame);
      convexPolygon.getClosestEdge(point.getPoint(), closestEdgeToPack.lineSegment);
   }

   public double getMaxX()
   {
      return convexPolygon.getMaxX();
   }

   public double getMinX()
   {
      return convexPolygon.getMinX();
   }

   public double getMaxY()
   {
      return convexPolygon.getMaxY();
   }

   public double getMinY()
   {
      return convexPolygon.getMinY();
   }

   public int getMinXMaxYIndex()
   {
      return convexPolygon.getMinXMaxYIndex();
   }

   public int getMinXMinYIndex()
   {
      return convexPolygon.getMinXMinYIndex();
   }

   public int getMaxXMaxYIndex()
   {
      return convexPolygon.getMaxXMaxYIndex();
   }

   public int getMaxXMinYIndex()
   {
      return convexPolygon.getMaxXMinYIndex();
   }

   public boolean isUpToDate()
   {
      return convexPolygon.isUpToDate();
   }

   public boolean isEmpty()
   {
      return convexPolygon.isEmpty();
   }

   public FramePoint2D[] getLineOfSightVerticesCopy(FramePoint2D observer)
   {
      checkReferenceFrameMatch(observer);
      FramePoint2D point1 = new FramePoint2D(getReferenceFrame(), convexPolygon.lineOfSightStartVertexCopy(observer.getPoint()));
      FramePoint2D point2 = new FramePoint2D(getReferenceFrame(), convexPolygon.lineOfSightEndVertexCopy(observer.getPoint()));
      return new FramePoint2D[] {point1, point2};
   }

}