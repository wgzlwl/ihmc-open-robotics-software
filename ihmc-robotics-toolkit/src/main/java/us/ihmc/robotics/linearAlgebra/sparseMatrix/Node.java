package us.ihmc.robotics.linearAlgebra.sparseMatrix;

import java.util.ArrayList;
import java.util.List;

public class Node
{
   private double value = Double.NaN;

   private NodeType type;

   private int rowStart;
   private int rowEnd;
   private int numRows;

   private int colStart;
   private int colEnd;
   private int numCols;

   private Quadrant childQuadrant;

   private final List<Node> leafNodesWithData = new ArrayList<>();

   Node(int rowStart, int rowEnd, int colStart, int colEnd)
   {
      this.rowStart = rowStart;
      this.rowEnd = rowEnd;
      this.colStart = colStart;
      this.colEnd = colEnd;
      numRows = rowEnd - rowStart;
      numCols = colEnd - colStart;

      type = NodeType.EMPTY;
   }

   public void reshape(int rowStart, int rowEnd, int colStart, int colEnd)
   {
      this.rowStart = rowStart;
      this.rowEnd = rowEnd;
      this.colStart = colStart;
      this.colEnd = colEnd;
      numRows = rowEnd - rowStart;
      numCols = colEnd - colStart;

      if (childQuadrant != null)
         childQuadrant.reshape(rowStart, rowEnd, colStart, colEnd);

      type = NodeType.EMPTY;
   }

   public void set(int row, int col, double value)
   {
      if (numCols > 0 || numRows > 0)
      {
         type = NodeType.POINTER;
         if (childQuadrant == null)
         {
            childQuadrant = createDataQuadrant();
         }
         childQuadrant.set(row, col, value);
      }
      else
      {
         type = NodeType.LEAF;
         this.value = value;
      }
   }

   public void clear()
   {
      value = Double.NaN;
      type = NodeType.EMPTY;
      if (childQuadrant != null)
         childQuadrant.clear();
   }

   public boolean isEmpty()
   {
      return type == NodeType.EMPTY;
   }

   /**
    * Returns the value at {@param row}, {@param col}.
    *
    * If this Node is a #NodeType.LEAF, it actually contains the value directly.
    * If not, it recurses from the children to the leaf to return the value.
    *
    * @param row row index
    * @param col col index
    * @return value
    */
   public double get(int row, int col)
   {
      if (type == NodeType.LEAF)
      {
         return value;
      }
      else
      {
         if (childQuadrant != null)
            return childQuadrant.get(row, col);
         else
            return Double.NaN;
      }
   }

   public List<Node> getLeafNodesWithData()
   {
      leafNodesWithData.clear();
      if (type == NodeType.LEAF)
      {
         leafNodesWithData.add(this);
      }
      else
      {
         leafNodesWithData.addAll(childQuadrant.getLeafNodesWithData());
      }

      return leafNodesWithData;
   }

   private Quadrant createDataQuadrant()
   {
      return new Quadrant(rowStart, colEnd, numRows, numCols);
   }
}
