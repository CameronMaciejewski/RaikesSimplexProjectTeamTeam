using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.Text;
using RaikesSimplexService.Contracts;
using RaikesSimplexService.DataModel;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace RaikesSimplexService.InsertTeamNameHere
{
    
    public class Solver : ISolver
    {
        int numVariables;
        int surplusVarCount;
        int slackVarCount;
        Matrix<double> BMatrix;
        Matrix<double> nahBMatrix;
        Matrix<double> LHS;
        Matrix<double> Binv;
        Matrix<double> RHSPrime;
        Matrix<double> Cb;
        Matrix<double>[] PMatrices;
        Matrix<double>[] PPrimeMatrices;
        double[] cPrime;
        Matrix<double> RHS;
        Matrix<double> objectiveRow;
        int[] BasicColumnIndices;
        int[] nonBasicColumnIndices;
        public Solution Solve(Model model)
        {
            generateInitialMatrices(model);
            DoPhase1();
            throw new NotImplementedException();
        }
        private void generateInitialMatrices(Model model)
        {
            Goal goal = model.Goal;
            var constraints = model.Constraints;
            numVariables = constraints[0].Coefficients.Length;
            surplusVarCount = 0;
            slackVarCount = 0;
            for (int i = 0; i < constraints.Count; i++)
            {
                if (constraints[i].Relationship.Equals(Relationship.LessThanOrEquals))
                {
                    slackVarCount += 1;
                }
                else if (constraints[i].Relationship.Equals(Relationship.GreaterThanOrEquals))
                {
                    surplusVarCount += 1;
                }
            }
            if (surplusVarCount != 0)
            {
                Console.Write("Z\t");
            }
            for (int i = 1; i <= numVariables; i++)
            {
                Console.Write("X" + i + "\t");
            }
            for (int i = 1; i <= (surplusVarCount + slackVarCount); i++)
            {
                Console.Write("S" + i + "\t");
            }
            for (int i = 1; i <= surplusVarCount; i++)
            {
                Console.Write("A" + i + "\t");
            }
            Console.WriteLine("\tRHS");

            int sCount = 1;
            int aCount = 1;
            List<int> rowsWithA = new List<int>();
            List<int> correspondingSlackVariable = new List<int>();
            int bMatrixBuildCounter = 0;
            int nahbMatrixBuildCounter = numVariables;
            int columns = numVariables + surplusVarCount * 2 + slackVarCount + 1;
            int rows = constraints.Count;
            if (surplusVarCount != 0)
            {
                rows += 1;
                BMatrix = Matrix<double>.Build.Dense(surplusVarCount + slackVarCount + 1, surplusVarCount + slackVarCount + 1, 0);
                BMatrix[surplusVarCount + slackVarCount, 0] = 1;
                nahBMatrix = Matrix<double>.Build.Dense(slackVarCount + surplusVarCount + 1, surplusVarCount + numVariables, 0);
            }
            else
            {
                BMatrix = Matrix<double>.Build.Dense(surplusVarCount + slackVarCount, surplusVarCount + slackVarCount, 0);
                nahBMatrix = Matrix<double>.Build.Dense(slackVarCount + surplusVarCount, surplusVarCount + numVariables, 0);
            }

            LHS = Matrix<double>.Build.Dense(rows, columns, 0);
            // each loop adds new row
            for (int i = 0; i < constraints.Count; i++)
            {
                // for the Z column
                if (surplusVarCount != 0)
                {
                    Console.Write("0\t");
                    LHS[i, 0] = 0;
                }
                for (int j = 0; j < constraints[i].Coefficients.Length; j++)
                {
                    Console.Write(constraints[i].Coefficients[j] + "\t");
                    LHS[i, j + 1] = constraints[i].Coefficients[j];
                    nahBMatrix[i, j] = constraints[i].Coefficients[j];
                }
                // loops through each remaining column on the left side
                for (int k = 1; k <= (surplusVarCount + slackVarCount); k++)
                {
                    if (k == sCount)
                    {
                        if (constraints[i].Relationship.Equals(Relationship.GreaterThanOrEquals))
                        {
                            Console.Write("-1\t");
                            LHS[i, k + numVariables] = -1;
                            nahBMatrix[i, nahbMatrixBuildCounter] = -1;
                            nahbMatrixBuildCounter += 1;
                        }
                        else if (constraints[i].Relationship.Equals(Relationship.LessThanOrEquals))
                        {
                            Console.Write("1\t");
                            LHS[i, k + numVariables] = 1;
                            BMatrix[i, bMatrixBuildCounter + 1] = 1;
                            bMatrixBuildCounter += 1;
                        }
                        else // if relationship is equal
                        {
                            Console.Write("0\t");
                        }
                    }
                    else
                    {
                        Console.Write("0\t");
                    }
                }

                for (int k = 1; k <= surplusVarCount; k++)
                {
                    if (k == aCount && constraints[i].Relationship.Equals(Relationship.GreaterThanOrEquals))
                    {
                        Console.Write("1\t");
                        LHS[i, k + numVariables + surplusVarCount + slackVarCount] = 1;
                        BMatrix[i, slackVarCount + k] = 1;
                        rowsWithA.Add(i);
                        correspondingSlackVariable.Add(sCount);
                    }
                    else
                    {
                        Console.Write("0\t");
                    }
                } // end loop through remaining LHS columns  
                if (constraints[i].Relationship.Equals(Relationship.GreaterThanOrEquals))
                {
                    sCount++;
                    aCount++;
                }
                else if (constraints[i].Relationship.Equals(Relationship.LessThanOrEquals))
                {
                    sCount++;
                }
                Console.WriteLine("\t" + constraints[i].Value);
            }

            // build the objective Row
            objectiveRow = Matrix<double>.Build.Dense(1, columns, 0);

            if (surplusVarCount != 0)
            {
                Console.Write("1\t");
                LHS[constraints.Count, 0] = 1;
                for (int i = 0; i < goal.Coefficients.Length; i++)
                {
                    Console.Write(goal.Coefficients[i] / -1 + "\t");
                    LHS[constraints.Count, i + 1] = goal.Coefficients[i] / -1;
                    nahBMatrix[constraints.Count, i] = goal.Coefficients[i] / -1;
                }
                for (int i = 0; i < (surplusVarCount * 2) + slackVarCount; i++)
                {
                    Console.Write("0\t");
                }
                Console.WriteLine("\t0\t");
                double[] totalCoefficient = new double[numVariables];

                for (int i = 0; i < rowsWithA.Count; i++)
                {
                    for (int j = 0; j < numVariables; j++)
                    {
                        totalCoefficient[j] += constraints[rowsWithA[i]].Coefficients[j];
                    }
                }
                Console.WriteLine("");
                // for the w matrix
                Console.Write("0\t");
                objectiveRow[0, 0] = 0;

                for (int i = 0; i < numVariables; i++)
                {
                    Console.Write(totalCoefficient[i] / -1 + "\t");
                    objectiveRow[0, i + 1] = totalCoefficient[i] / -1;
                }
                //loops through each S variable column
                for (int i = numVariables + 1; i <= numVariables + surplusVarCount + slackVarCount; i++)
                {
                    // if there is a slack variable
                    if (correspondingSlackVariable.Contains(i - numVariables))
                    {
                        Console.Write("1\t");
                        objectiveRow[0, i] = 1;
                    }
                    else
                    {
                        Console.Write("0\t");
                    }
                }
                for (int i = 0; i < surplusVarCount; i++)
                {
                    Console.Write("0\t");
                }
            }
            else
            {
                for (int i = 0; i < goal.Coefficients.Length; i++)
                {
                    Console.Write(goal.Coefficients[i] / -1 + "\t");
                    objectiveRow[0, i] = goal.Coefficients[i] / -1;
                }
                for (int i = 0; i < (surplusVarCount * 2) + slackVarCount; i++)
                {
                    Console.Write("0\t");
                }
            }
            // Build the RHS matrix
            if (surplusVarCount != 0)
            {
                RHS = Matrix<double>.Build.Dense(constraints.Count + 1, 1, 0);
            }
            else
            {
                RHS = Matrix<double>.Build.Dense(constraints.Count, 1, 0);
            }
            for (int i = 0; i < constraints.Count; i++)
            {
                RHS[i, 0] = constraints[i].Value;
            }

            Console.WriteLine("\t Objective Row");
            Console.WriteLine(LHS.ToString());
            Console.WriteLine(objectiveRow.ToString());
            Console.WriteLine(RHS.ToString());
            Console.WriteLine(BMatrix.ToString());
            Console.WriteLine(nahBMatrix.ToString());
        }
        private void DoPhase1()
        {
            Binv = BMatrix.Inverse();
            Console.WriteLine(Binv.ToString());
            Cb = Matrix<double>.Build.Dense(1, BMatrix.ColumnCount);
            PMatrices = new Matrix<double>[numVariables + surplusVarCount];
            PPrimeMatrices = new Matrix<double>[numVariables + surplusVarCount];
            cPrime = new double[nahBMatrix.ColumnCount];
            BasicColumnIndices = new int[BMatrix.ColumnCount];
            nonBasicColumnIndices = new int[nahBMatrix.ColumnCount];
            //loop through objective row
            int basicCounter = 0;
            int nonBasicCounter = 0;
            for (int i = 0; i < objectiveRow.ColumnCount; i++){
                
                if (objectiveRow[0,i] == 0){
                    BasicColumnIndices[basicCounter] = i;
                    basicCounter += 1;
                } 
                else
                {
                    nonBasicColumnIndices[nonBasicCounter] = i;
                    nonBasicCounter += 1;
                }
            }
            for (int i = 0; i < BMatrix.ColumnCount; i++)
            {

            }
            for (int i = 0; i < nahBMatrix.ColumnCount; i++)
            {
                PMatrices[i] = nahBMatrix.Column(i).ToColumnMatrix();
                PPrimeMatrices[i] = Binv * PMatrices[i];
                Console.WriteLine(PPrimeMatrices[i].ToString());
                cPrime[i] = objectiveRow[0, nonBasicColumnIndices[i]] - (Cb * PPrimeMatrices[i])[0,0];
            }
            RHSPrime = Binv * RHS;
            Console.WriteLine(RHSPrime.ToString());
            int indexOfMinCPrime = Array.IndexOf(cPrime, cPrime.Min());
            int smallestRatioIndex = getSmallestRatio(RHSPrime, PPrimeMatrices[indexOfMinCPrime]);
            swapColumns(indexOfMinCPrime, smallestRatioIndex);
            int hi = 2;
        }
        private int getSmallestRatio(Matrix<double> tempRHSPrime, Matrix<double> tempPPrime)
        {
            double minRatio = 10000000000000000000;
            int minIndex = 0;
            for (int i = 0; i < tempRHSPrime.RowCount; i++)
            {
                double ratio = tempRHSPrime[i, 0] / tempPPrime[i, 0];
                if (ratio < minRatio && ratio > 0)
                {
                    minRatio = ratio;
                    minIndex = i;
                }
            }
            return minIndex;
        }
        private void swapColumns(int enteringNahBIndex, int leavingBIndex)
        {
            //swap b and nahB
            Matrix<double> temp = Matrix<double>.Build.Dense(BMatrix.RowCount, 1);
            for (int i = 0; i < BMatrix.RowCount; i++)
            {
                temp[i,0] = BMatrix[i, leavingBIndex];
                BMatrix[i, leavingBIndex] = nahBMatrix[i, enteringNahBIndex];
                nahBMatrix[i, enteringNahBIndex] = temp[i, 0];
            }
            //swap LHS rows
            for (int i = 0; i < BMatrix.RowCount; i++)
            {
                temp[i, 0] = LHS[i, BasicColumnIndices[leavingBIndex]];
                LHS[i, BasicColumnIndices[leavingBIndex]] = LHS[i, nonBasicColumnIndices[enteringNahBIndex]];
                LHS[i, nonBasicColumnIndices[enteringNahBIndex]] = temp[i, 0];
            }
            //swap objective row columns
            double objectiveTemp;
            objectiveTemp = objectiveRow[0, BasicColumnIndices[leavingBIndex]];
            objectiveRow[0, BasicColumnIndices[leavingBIndex]] = objectiveRow[0, nonBasicColumnIndices[enteringNahBIndex]];
            objectiveRow[0, nonBasicColumnIndices[enteringNahBIndex]] = objectiveTemp;
        }
    }
}
