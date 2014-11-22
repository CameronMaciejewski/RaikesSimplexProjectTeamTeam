using RaikesSimplexService.InsertTeamNameHere;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using RaikesSimplexService.DataModel;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace UnitTests
{
    
    
    /// <summary>
    ///This is a test class for SolverTest and is intended
    ///to contain all SolverTest Unit Tests
    ///</summary>
    [TestClass()]
    public class SolverTest
    {
        private TestContext testContextInstance;

        /// <summary>
        ///Gets or sets the test context which provides
        ///information about and functionality for the current test run.
        ///</summary>
        public TestContext TestContext
        {
            get
            {
                return testContextInstance;
            }
            set
            {
                testContextInstance = value;
            }
        }

        #region Additional test attributes
        // 
        //You can use the following additional attributes as you write your tests:
        //
        //Use ClassInitialize to run code before running the first test in the class
        //[ClassInitialize()]
        //public static void MyClassInitialize(TestContext testContext)
        //{
        //}
        //
        //Use ClassCleanup to run code after all tests in a class have run
        //[ClassCleanup()]
        //public static void MyClassCleanup()
        //{
        //}
        //
        //Use TestInitialize to run code before running each test
        //[TestInitialize()]
        //public void MyTestInitialize()
        //{
        //}
        //
        //Use TestCleanup to run code after each test has run
        //[TestCleanup()]
        //public void MyTestCleanup()
        //{
        //}
        //
        #endregion


        /// <summary>
        ///A test for Solve
        ///</summary>
        [TestMethod()]
        public void ExampleSolveTest()
        {
            #region Arrange
            var target = new Solver();            

            var lc1 = new LinearConstraint()
            {
                Coefficients = new double[2] { 8, 12 },
                Relationship = Relationship.GreaterThanOrEquals,
                Value = 24
            };

            var lc2 = new LinearConstraint()
            {
                Coefficients = new double[2] { 12, 12 },
                Relationship = Relationship.GreaterThanOrEquals,
                //poop
                Value = 36
            };

            var lc3 = new LinearConstraint()
            {
                Coefficients = new double[2] { 2, 1 },
                Relationship = Relationship.GreaterThanOrEquals,
                Value = 4
            };

            var lc4 = new LinearConstraint()
            {
                Coefficients = new double[2] { 1, 1 },
                Relationship = Relationship.LessThanOrEquals,
                Value = 5
            };

            var constraints = new List<LinearConstraint>() {lc1, lc2, lc3, lc4};

            var goal = new Goal() 
            { 
                Coefficients = new double[2] { 0.2, 0.3 },
                ConstantTerm = 0
            };           

            var model = new Model()
            {
                Constraints = constraints,
                Goal = goal,
                GoalKind = GoalKind.Minimize
            };
            
            var expected = new Solution()
            {
                Decisions = new double[2] { 3, 0 },
                Quality = SolutionQuality.Optimal,
                AlternateSolutionsExist = false,
                OptimalValue = 0.6
            };
            #endregion
            printModelInfo(model);
            printModelMatrix(model);
            ////Act
            //var actual = target.Solve(model);

            ////Assert
            //CollectionAssert.AreEqual(expected.Decisions, actual.Decisions);
            //Assert.AreEqual(expected.Quality, actual.Quality);
            //Assert.AreEqual(expected.AlternateSolutionsExist, actual.AlternateSolutionsExist);
        }

        public void printModelInfo(Model model)
        {
            Goal goal = model.Goal;
            var constraints = model.Constraints;

            System.Diagnostics.Debug.Write("Objective Funcion: " + model.GoalKind + " Z = ");


            for (int i = 0; i < goal.Coefficients.Length ; i++)
            {
                System.Diagnostics.Debug.Write("(" + goal.Coefficients[i] + ")X" + (i + 1) + " + ");
            }
            System.Diagnostics.Debug.WriteLine("(" + goal.ConstantTerm + ")");

            System.Diagnostics.Debug.WriteLine("Constraints: ");
            for (int i = 0; i < constraints.Count; i++)
            {
                var constraint = constraints[i];
                for (int j = 0; j < constraint.Coefficients.Length -1; j++ )
                {
                    System.Diagnostics.Debug.Write("(" + constraint.Coefficients[j] + ")X" + (j + 1) + " + ");
                }
                System.Diagnostics.Debug.Write("(" + constraint.Coefficients[constraint.Coefficients.Length - 1] + ")X" + (constraint.Coefficients.Length));
                if (constraint.Relationship.Equals(Relationship.Equals))
                {
                    System.Diagnostics.Debug.WriteLine(" = " + constraint.Value);
                }
                else if (constraint.Relationship.Equals(Relationship.GreaterThanOrEquals))
                {
                    System.Diagnostics.Debug.WriteLine(" >= " + constraint.Value);
                }
                else // Relationship is less than or equals
                {
                    System.Diagnostics.Debug.WriteLine(" <= " + constraint.Value);
                }
            }
            System.Diagnostics.Debug.WriteLine("");
        }

        public void printModelMatrix(Model model)
        {
            Goal goal = model.Goal;
            var constraints = model.Constraints;
            int numVariables = constraints[0].Coefficients.Length;
            int surplusVarCount = 0;
            int slackVarCount = 0;
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
                System.Diagnostics.Debug.Write("Z\t");
            }
            for (int i = 1; i <= numVariables; i++)
            {
                System.Diagnostics.Debug.Write("X" + i + "\t");
            }
            for (int i = 1; i <= (surplusVarCount + slackVarCount); i++) 
            {
                System.Diagnostics.Debug.Write("S" + i + "\t");
            }
            for (int i = 1; i <= surplusVarCount; i++)
            {
                System.Diagnostics.Debug.Write("A" + i + "\t");
            }
            System.Diagnostics.Debug.WriteLine("\tRHS");

            int sCount = 1;
            int aCount = 1;
            List<int> rowsWithA = new List<int>();
            List<int> correspondingSlackVariable = new List<int>();
            Matrix<double> LHS;
            int columns = numVariables + surplusVarCount * 2 + slackVarCount;
            int rows = constraints.Count;
            LHS = Matrix<double>.Build.Dense(rows, columns, 0);
            // each loop adds new row
            for (int i = 0; i < constraints.Count; i++)
            {
                // for the Z column
                System.Diagnostics.Debug.Write("0\t");
                for (int j = 0; j < constraints[i].Coefficients.Length; j++)
                {
                    System.Diagnostics.Debug.Write(constraints[i].Coefficients[j] + "\t");
                    LHS[i, j] = constraints[i].Coefficients[j];
                }
                // loops through each remaining column on the left side
                for (int k = 1; k <= (surplusVarCount  + slackVarCount ); k++)
                {
                    if (k == sCount)
                    {
                        if (constraints[i].Relationship.Equals(Relationship.GreaterThanOrEquals))
                        {
                            System.Diagnostics.Debug.Write("-1\t");                          
                        }
                        else if (constraints[i].Relationship.Equals(Relationship.LessThanOrEquals))
                        {
                            System.Diagnostics.Debug.Write("1\t");                   
                        }else // if relationship is equal
                        {
                            System.Diagnostics.Debug.Write("0\t");
                        }
                    }
                    else
                    {
                        System.Diagnostics.Debug.Write("0\t");
                    }
                }
                for (int k = 1; k <= surplusVarCount; k++)
                {               
                    if (k == aCount)
                    {
                        System.Diagnostics.Debug.Write("1\t");
                        rowsWithA.Add(i);
                        correspondingSlackVariable.Add(sCount);
                    }
                    else
                    {
                        System.Diagnostics.Debug.Write("0\t");
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
                System.Diagnostics.Debug.WriteLine("\t" + constraints[i].Value);
            }
            // if there is a Z row
            if (surplusVarCount != 0)
            {
                System.Diagnostics.Debug.Write("1\t");
                for (int i = 0; i < goal.Coefficients.Length; i++)
                {
                    System.Diagnostics.Debug.Write(goal.Coefficients[i]/-1 + "\t");
                }
                for (int i = 0; i < (surplusVarCount * 2) + slackVarCount; i++)
                {
                    System.Diagnostics.Debug.Write("0\t");
                }
                System.Diagnostics.Debug.WriteLine("\t0\t");
            }

            double[] totalCoefficient = new double[numVariables];
            
            for (int i = 0; i < rowsWithA.Count; i++)
            {
                for (int j = 0; j < numVariables; j++)
                {
                    totalCoefficient[j] += constraints[rowsWithA[i]].Coefficients[j];
                }
            }
            System.Diagnostics.Debug.WriteLine("");
            // for the w matrix
            if (surplusVarCount != 0)
            {
                System.Diagnostics.Debug.Write("0\t");
            }
            for (int i = 0; i < numVariables; i++)
            {
                System.Diagnostics.Debug.Write(totalCoefficient[i] + "\t");
            }
            //loops through each S variable column
            for (int i = 1; i <= surplusVarCount + slackVarCount; i++)
            {
                // if there is a slack variable
                if (correspondingSlackVariable.Contains(i))
                {
                    System.Diagnostics.Debug.Write("-1\t");
                }
                else
                {
                    System.Diagnostics.Debug.Write("0\t");
                }
            }
            for (int i = 0; i < surplusVarCount; i++)
            {
                System.Diagnostics.Debug.Write("0\t");
            }
            System.Diagnostics.Debug.WriteLine("\t Objective Row");
            
        }

    }
}
