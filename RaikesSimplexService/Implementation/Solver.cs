﻿using System;
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
        int[] basicVariables;
        double[] cPrime;
        Matrix<double> RHS;
        Matrix<double> objectiveRow;
        int[] BasicColumnIndices;
        int[] nonBasicColumnIndices;
        SolutionQuality quality;
        long timeOutCounter;
        public Solution Solve(Model model)
        {
            timeOutCounter = 0;
            quality = SolutionQuality.Optimal;
            generateInitialMatrices(model);
            if (surplusVarCount != 0)
            {                
                generateNewIndices();
                do
                {                   
                    Do2Phase();
                    if (timeOutCounter >= 10000000000)
                    {
                        quality = SolutionQuality.TimedOut;
                    }
                    printAllTheThings();
                } while (cPrimesAreNegative() && quality == SolutionQuality.Optimal);
                simplifyLHS();
            }
            // Default optimal value and decisions
            double optimalValue = 0;
            double[] decisions = new double[numVariables];
            for (int i = 0; i < decisions.Length; i++)
            {
                decisions[i] = 0;
            }
            // if solution quality has already been changed
            if (quality == SolutionQuality.Optimal)
            {
                Console.WriteLine("LHS Matrix After 2 phase stuff");
                Console.WriteLine(LHS.ToString());
                generate1PhaseMatrices();
                do
                {
                    Do1Phase();
                    if (timeOutCounter >= 10000000000)
                    {
                        quality = SolutionQuality.TimedOut;
                    }
                } while (cPrimesAreNegative() && quality == SolutionQuality.Optimal);
                if (quality == SolutionQuality.Optimal)
                {
                    for (int i = 0; i < basicVariables.Length; i++)
                    {
                        if (basicVariables[i] < numVariables)
                        {
                            decisions[basicVariables[i]] = RHSPrime[i, 0];
                        }
                    }
                    for (int i = 0; i < decisions.Length; i++)
                    {
                        optimalValue += decisions[i] * model.Goal.Coefficients[i];
                    }
                }             
            }
            
            
            bool alternateSolutionsExist = false; //sorry, running out of time.            
            Solution solution = new Solution { AlternateSolutionsExist = alternateSolutionsExist, Decisions = decisions, OptimalValue = optimalValue, Quality = quality };
            return solution;
        }
        private void printAllTheThings()
        {
            Console.WriteLine("LHS Matrix");
            Console.WriteLine(LHS.ToString());
            Console.WriteLine("Objective Row");
            Console.WriteLine(objectiveRow.ToString());
            Console.WriteLine("RHS Matrix");
            Console.WriteLine(RHS.ToString());
            Console.WriteLine("B Matrix");
            Console.WriteLine(BMatrix.ToString());
            Console.WriteLine("NAHB Matrix");
            Console.WriteLine(nahBMatrix.ToString());
            Console.WriteLine("CB Matrix");
            Console.WriteLine(Cb.ToString());
            Console.WriteLine("CPrime Array");
            for (int i = 0; i < cPrime.Length; i++)
            {
                Console.Write(cPrime[i] + "\t");
            }
            Console.WriteLine();
        }
        private void generateInitialMatrices(Model model)
        {
            Goal goal = model.Goal;
            var constraints = model.Constraints;
            int minMaxEditor = 1;
            if (model.GoalKind == GoalKind.Maximize)
            {
                minMaxEditor = -1;
            }
            numVariables = constraints[0].Coefficients.Length;
            surplusVarCount = 0;
            slackVarCount = 0;
            basicVariables = new int[constraints.Count];
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
                            if (surplusVarCount != 0)
                            {
                                BMatrix[i, bMatrixBuildCounter + 1] = 1;
                            }
                            else
                            {
                                BMatrix[i, bMatrixBuildCounter] = 1;
                            }
                            
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
            

            if (surplusVarCount != 0)
            {
                objectiveRow = Matrix<double>.Build.Dense(1, columns, 0);
                Console.Write("1\t");
                LHS[constraints.Count, 0] = 1;
                for (int i = 0; i < goal.Coefficients.Length; i++)
                {
                    Console.Write(goal.Coefficients[i] / minMaxEditor + "\t");
                    LHS[constraints.Count, i + 1] = goal.Coefficients[i] / minMaxEditor;
                    nahBMatrix[constraints.Count, i] = goal.Coefficients[i] / minMaxEditor;
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
                objectiveRow = Matrix<double>.Build.Dense(1, columns -1, 0);
                for (int i = 0; i < goal.Coefficients.Length; i++)
                {
                    Console.Write(goal.Coefficients[i] / minMaxEditor + "\t");
                    objectiveRow[0, i] = goal.Coefficients[i] / minMaxEditor;
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
        }
        private void generateNewIndices()
        {
            BasicColumnIndices = new int[BMatrix.ColumnCount];
            nonBasicColumnIndices = new int[nahBMatrix.ColumnCount];
            //loop through objective row
            int basicCounter = 0;
            int nonBasicCounter = 0;
            for (int i = 0; i < LHS.ColumnCount; i++)
            {
                int numOnes = 0;
                int numZeros = 0;
                for (int j = 0; j < LHS.RowCount; j++)
                {
                    if (LHS[j, i] == 1)
                    {
                        numOnes += 1;
                    }
                    if (LHS[j, i] == 0)
                    {
                        numZeros += 1;
                    }
                }
                if (numOnes != 1 || numZeros != LHS.RowCount - 1)
                {
                    nonBasicColumnIndices[nonBasicCounter] = i;
                    nonBasicCounter += 1;                   
                }
                else
                {
                    BasicColumnIndices[basicCounter] = i;
                    basicCounter += 1;
                }
            }
            //for (int i = 0; i < LHS.ColumnCount; i++)
            //{

            //    if (objectiveRow[0, i] == 0)
            //    {
            //        BasicColumnIndices[basicCounter] = i;
            //        basicCounter += 1;
            //    }
            //    else
            //    {
            //        nonBasicColumnIndices[nonBasicCounter] = i;
            //        nonBasicCounter += 1;
            //    }
            //}
        }
        private void Do2Phase()
        {
            Binv = BMatrix.Inverse();
            Console.WriteLine(Binv.ToString());
            Cb = Matrix<double>.Build.Dense(1, BMatrix.ColumnCount);
            PMatrices = new Matrix<double>[numVariables + surplusVarCount];
            PPrimeMatrices = new Matrix<double>[numVariables + surplusVarCount];
            cPrime = new double[nahBMatrix.ColumnCount];
            
            for (int i = 0; i < BMatrix.ColumnCount; i++)
            {
                Cb[0, i] = objectiveRow[0, BasicColumnIndices[i]];
            }
            for (int i = 0; i < nahBMatrix.ColumnCount; i++)
            {
                PMatrices[i] = nahBMatrix.Column(i).ToColumnMatrix();
                PPrimeMatrices[i] = Binv * PMatrices[i];
                cPrime[i] = objectiveRow[0, nonBasicColumnIndices[i]] - (Cb * PPrimeMatrices[i])[0,0];
            }
            RHSPrime = Binv * RHS;
            int indexOfMinCPrime = Array.IndexOf(cPrime, cPrime.Min());
            int smallestRatioIndex = getSmallestRatio(RHSPrime, PPrimeMatrices[indexOfMinCPrime]);
            swapColumns(indexOfMinCPrime, smallestRatioIndex);
            timeOutCounter += 1;
        }
        private Boolean cPrimesAreNegative()
        {
            for (int i = 0; i < cPrime.Length; i++)
            {
                if (cPrime[i] < 0)
                {
                    return true;
                }
            }
            return false;
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
            if (minRatio == 10000000000000000000)
            {
                quality = SolutionQuality.Unbounded;
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
            //for (int i = 0; i < BMatrix.RowCount; i++)
            //{
            //    temp[i, 0] = LHS[i, BasicColumnIndices[leavingBIndex]];
            //    LHS[i, BasicColumnIndices[leavingBIndex]] = LHS[i, nonBasicColumnIndices[enteringNahBIndex]];
            //    LHS[i, nonBasicColumnIndices[enteringNahBIndex]] = temp[i, 0];
            //}
            //swap objective row columns
            //double objectiveTemp;
            //objectiveTemp = objectiveRow[0, BasicColumnIndices[leavingBIndex]];
            //objectiveRow[0, BasicColumnIndices[leavingBIndex]] = objectiveRow[0, nonBasicColumnIndices[enteringNahBIndex]];
            //objectiveRow[0, nonBasicColumnIndices[enteringNahBIndex]] = objectiveTemp;
            //swap index matrices
            int tempIndex;
            tempIndex = BasicColumnIndices[leavingBIndex];
            BasicColumnIndices[leavingBIndex] = nonBasicColumnIndices[enteringNahBIndex];
            nonBasicColumnIndices[enteringNahBIndex] = tempIndex;           
        }
        private void simplifyLHS()
        {
            LHS = Binv * LHS;
            for ( int i = 0; i < LHS.ColumnCount; i++)
            {
                int onesCounter = 0;
                int zerosCounter = 0;
                for (int j = 0; j < LHS.RowCount; j++ )
                {
                    if (LHS[j, i] < .00001 && LHS[j, i] > -0.00001)
                    {
                        LHS[j, i] = 0;
                    }
                    LHS[j, i] = Math.Round(LHS[j, i], 6);
                    //check for infeasable
                    if (LHS[j, i] == 0)
                    {
                        zerosCounter += 1;
                    }
                    if (LHS[j,i] == 1)
                    {
                        onesCounter += 1;
                    }

                }
                // check for infeasable
                if (i >= LHS.ColumnCount - surplusVarCount)
                {
                    if (onesCounter == 1 && zerosCounter == LHS.RowCount - 1)
                    {
                        quality = SolutionQuality.Infeasible;
                    }
                }
                
            }
            Console.WriteLine(LHS.ToString());
            int originalColumnCount = LHS.ColumnCount;
            for (int i = originalColumnCount; i > originalColumnCount - surplusVarCount; i-- )
            {
                LHS = LHS.RemoveColumn(i - 1);
            }
            
            RHS = Binv * RHS;
            Console.WriteLine(LHS.Row(0).ToString());
            objectiveRow = LHS.Row(0).ToRowMatrix();
            objectiveRow = objectiveRow.RemoveColumn(0);
            LHS = LHS.RemoveRow(0);
            RHS = RHS.RemoveRow(0);
        }
        private void Do1Phase()
        {
                       
            Binv = BMatrix.Inverse();
            Cb = Matrix<double>.Build.Dense(1, BMatrix.ColumnCount);
            PMatrices = new Matrix<double>[nahBMatrix.ColumnCount];
            PPrimeMatrices = new Matrix<double>[nahBMatrix.ColumnCount];
            cPrime = new double[nahBMatrix.ColumnCount];

            for (int i = 0; i < BMatrix.ColumnCount; i++)
            {
                Cb[0, i] = objectiveRow[0, BasicColumnIndices[i]];
            }
            for (int i = 0; i < nahBMatrix.ColumnCount; i++)
            {
                PMatrices[i] = nahBMatrix.Column(i).ToColumnMatrix();
                PPrimeMatrices[i] = Binv * PMatrices[i];
                cPrime[i] = objectiveRow[0, nonBasicColumnIndices[i]] - (Cb * PPrimeMatrices[i])[0, 0];
            }

            RHSPrime = Binv * RHS;
            int indexOfMinCPrime = Array.IndexOf(cPrime, cPrime.Min());
            int smallestRatioIndex = getSmallestRatio(RHSPrime, PPrimeMatrices[indexOfMinCPrime]);
            if (cPrimesAreNegative())
            {
                basicVariables[smallestRatioIndex] = nonBasicColumnIndices[indexOfMinCPrime];
            }
            swapColumns(indexOfMinCPrime, smallestRatioIndex);
            timeOutCounter += 1;

        }
        private void generate1PhaseMatrices()
        {
            LHS = LHS.RemoveColumn(0);
            List<int> BMatrixColumnIndices = new List<int>();
            List<int> nahBMatrixColumnIndices = new List<int>();
            for (int i = 0; i < LHS.ColumnCount; i++)
            {
                int numOnes = 0;
                int numZeros = 0;
                for (int j = 0; j < LHS.RowCount; j++)
                {
                    if (LHS[j, i] == 1)
                    {
                        numOnes += 1;
                    }
                    if (LHS[j, i] == 0)
                    {
                        numZeros += 1;
                    }
                }
                if (numOnes != 1 || numZeros != LHS.RowCount - 1)
                {
                    nahBMatrixColumnIndices.Add(i);
                }
                else
                {
                    BMatrixColumnIndices.Add(i);
                }
            }
            BasicColumnIndices = new int[BMatrixColumnIndices.Count];
            nonBasicColumnIndices = new int[nahBMatrixColumnIndices.Count];
            Matrix<double> newBMatrix = Matrix<double>.Build.Dense(LHS.RowCount, BMatrixColumnIndices.Count);
            Matrix<double> newNahBMatrix = Matrix<double>.Build.Dense(LHS.RowCount, nahBMatrixColumnIndices.Count);
            for (int i = 0; i < BMatrixColumnIndices.Count; i++)
            {
                for (int j = 0; j < LHS.RowCount; j++)
                {
                    newBMatrix[j, i] = LHS[j, BMatrixColumnIndices[i]];
                }
                basicVariables[i] = BMatrixColumnIndices[i];
                BasicColumnIndices[i] = BMatrixColumnIndices[i];
            }
            for (int i = 0; i < nahBMatrixColumnIndices.Count; i++)
            {
                for (int j = 0; j < LHS.RowCount; j++)
                {
                    newNahBMatrix[j, i] = LHS[j, nahBMatrixColumnIndices[i]];
                }
                nonBasicColumnIndices[i] = nahBMatrixColumnIndices[i];
            }
            BMatrix = newBMatrix;
            nahBMatrix = newNahBMatrix;
        }
    }
}
