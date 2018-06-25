package us.ihmc.robotics.linearAlgebra.sparseMatrix;

public class SparseMatrix
{
   private int numRows;
   private int numCols;

   private final Node rootNode;

   public SparseMatrix(int numRows, int numCols)
   {
      this.numRows = numRows;
      this.numCols = numCols;

      rootNode = new Node(0, numRows - 1, 0, numCols - 1);
   }

   public void set(int row, int col, double value)
   {
      rootNode.set(row, col, value);
   }

   public double get(int row, int col)
   {
      return rootNode.get(row, col);
   }
}
