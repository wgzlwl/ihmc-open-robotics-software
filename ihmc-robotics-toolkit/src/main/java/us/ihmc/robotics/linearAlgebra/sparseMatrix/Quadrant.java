package us.ihmc.robotics.linearAlgebra.sparseMatrix;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class Quadrant
{
   private enum QuadrantKey
   {
      TopLeft, TopRight, BottomLeft, BottomRight;

      public static QuadrantKey[] values = values();
   }

   private final EnumMap<QuadrantKey, Node> nodes = new EnumMap<>(QuadrantKey.class);
   private final EnumMap<QuadrantKey, Node> unusedNodes = new EnumMap<>(QuadrantKey.class);

   private int rowStart;
   private int colStart;
   private int numRows;
   private int numCols;

   private int numTopRows;
   private int numBottomRows;
   private int numLeftCols;
   private int numRightCols;

   Quadrant(int rowStart, int rowEnd, int colStart, int colEnd)
   {
      for (QuadrantKey quadrant : QuadrantKey.values)
         unusedNodes.put(quadrant, new Node(0, 0, 0, 0));

      reshape(rowStart, rowEnd, colStart, colEnd);
   }

   private void assembleQuadrant()
   {
      for (QuadrantKey quadrant : QuadrantKey.values)
      {
         if (nodes.containsKey(quadrant))
            unusedNodes.put(quadrant, nodes.remove(quadrant));
      }

      if (numTopRows > 0)
      {
         if (numLeftCols > 0)
         {
            int startRow = rowStart;
            int endRow = rowStart + numTopRows;
            int startCol = colStart;
            int endCol = colStart + numLeftCols;
            Node node = unusedNodes.remove(QuadrantKey.TopLeft);
            node.reshape(startRow, endRow, startCol, endCol);
            nodes.put(QuadrantKey.TopLeft, node);
         }

         if (numRightCols > 0)
         {
            int startRow = rowStart;
            int endRow = rowStart + numTopRows;
            int startCol = colStart + numLeftCols;
            int endCol = colStart + numCols;
            Node node = unusedNodes.remove(QuadrantKey.TopRight);
            node.reshape(startRow, endRow, startCol, endCol);
            nodes.put(QuadrantKey.TopRight, node);
         }
      }
      if (numBottomRows > 0)
      {
         if (numLeftCols > 0)
         {
            int startRow = rowStart + numTopRows;
            int endRow = rowStart + numRows;
            int startCol = colStart;
            int endCol = colStart + numLeftCols;
            Node node = unusedNodes.remove(QuadrantKey.BottomLeft);
            node.reshape(startRow, endRow, startCol, endCol);
            nodes.put(QuadrantKey.BottomLeft, node);
         }
         if (numRightCols > 0)
         {
            int startRow = rowStart + numTopRows;
            int endRow = rowStart + numRows;
            int startCol = colStart + numLeftCols;
            int endCol = colStart + numCols;
            Node node = unusedNodes.remove(QuadrantKey.BottomRight);
            node.reshape(startRow, endRow, startCol, endCol);
            nodes.put(QuadrantKey.BottomRight, node);
         }
      }
   }

   public void clear()
   {
      for (QuadrantKey quadrant : QuadrantKey.values)
      {
         Node node = nodes.get(quadrant);
         if (node != null)
            node.clear();
      }
   }

   public void set(int row, int col, double value)
   {
      nodes.get(getQuadrant(row, col)).set(row, col, value);
   }

   private QuadrantKey getQuadrant(int row, int col)
   {
      if (row < rowStart || row > rowStart + numRows)
         throw new IllegalArgumentException("Row " + row + " is out of bounds [" + rowStart + ", " + rowStart + numRows + "].");
      if (col < colStart || col > colStart + numCols)
         throw new IllegalArgumentException("Col " + col + " is out of bounds [" + colStart + ", " + colStart + numCols + "].");

      if (row < rowStart + numTopRows)
      {
         if (col < colStart + numLeftCols)
            return QuadrantKey.TopLeft;
         else
            return QuadrantKey.TopRight;
      }
      else
      {
         if (col < colStart + numLeftCols)
            return QuadrantKey.BottomLeft;
         else
            return QuadrantKey.BottomRight;
      }
   }

   public double get(int row, int col)
   {
      return nodes.get(getQuadrant(row, col)).get(row, col);
   }

   public boolean isEmpty()
   {
      for (QuadrantKey quadrant : QuadrantKey.values)
      {
         Node node = nodes.get(quadrant);
         if (node != null)
         {
            if (!node.isEmpty())
               return false;
         }
      }

      return true;
   }

   public void reshape(int rowStart, int rowEnd, int colStart, int colEnd)
   {
      this.rowStart = rowStart;
      this.colStart = colStart;
      this.numRows = rowEnd - rowStart;
      this.numCols = colEnd - colStart;

      numBottomRows = Math.floorDiv(numRows, 2);
      numTopRows = numRows - numBottomRows;
      numRightCols = Math.floorDiv(numCols, 2);
      numLeftCols = numCols - numRightCols;

      assembleQuadrant();
   }


   private final List<Node> leafNodesWithData = new ArrayList<>();
   public List<Node> getLeafNodesWithData()
   {
      leafNodesWithData.clear();
      for (QuadrantKey quadrant : QuadrantKey.values)
      {
         if (nodes.containsKey(quadrant))
            leafNodesWithData.addAll(nodes.get(quadrant).getLeafNodesWithData());
      }

      return leafNodesWithData;
   }
}
