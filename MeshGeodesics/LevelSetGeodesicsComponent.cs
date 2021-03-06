﻿using System;
using System.Collections.Generic;
using System.Windows.Forms;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

using Cureos.Numerics.Optimizers;
using LibOptimization.Optimization;
using LibOptimization.Util;
using LibOptimization;

namespace MeshGeodesics
{
    
    /// <summary>
    /// Mesh geodesics component.
    /// </summary>
    public class MeshGeodesicsComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public MeshGeodesicsComponent()
          : base("Level Sets Geodesics", "LS Geod",
            "Generate geodesics by level set method",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Initial Values", "V", "OPTIONAL Initial Values per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Width", "W", "Desired separation between geodesics", GH_ParamAccess.item);
            pManager.AddNumberParameter("Lambda", "lmbd", "Regularization weight", GH_ParamAccess.item);
            pManager.AddNumberParameter("Nu", "nu", "Equal width weight", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Max Iter", "Iter", "Maximum Iterations for the Vector Field Optimization.\nCareful with this setting!! It might take a while to finish...", GH_ParamAccess.item, 500);
            pManager.AddIntegerParameter("Optimizer", "Opti", "Select desired optimizer: 0=BOBYQA 1=NelderMead 2=SimulatedAnnealing 3=Firefly",GH_ParamAccess.item,0);
            pManager.AddNumberParameter("Optimization Objective", "Obj", "Numerical objective to stop the optimization process", GH_ParamAccess.item, 10);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Pattern", "P", "Resulting level set curve pattern", GH_ParamAccess.tree);
            pManager.AddVectorParameter("Gradient", "grad", "Gradient of scalar field per face", GH_ParamAccess.list);
            pManager.AddNumberParameter("Divergence", "div", "Divergence of the gradient per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Voronoi Area", "A", "Voronoi Area of each vertex of the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("MinimizationValue", "Min", "Minimization value", GH_ParamAccess.item);
            pManager.AddNumberParameter("Function values", "Values", "Resulting scalar function values", GH_ParamAccess.list);
            pManager.AddNumberParameter("Normalized Divergence", "Kg", "Geodesic curvature per vertex or: the divergence of the normalized gradient of the scalar function", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = new Mesh();
            List<double> initialValues = new List<double>();
            double desiredWidth = 0.0;
            double lambda = 0.0;
            double nu = 0.0;
            int maxCalls = 0;
            int optim = 0;
            double objective = 10;
            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetDataList(1, initialValues)) return;
            if (!DA.GetData(2, ref desiredWidth)) return;
            if (!DA.GetData(3, ref lambda)) return;
            if (!DA.GetData(4, ref nu)) return;
            if (!DA.GetData(5, ref maxCalls)) return;
            if (!DA.GetData(6, ref optim)) return;
            if (!DA.GetData(7, ref objective)) return;

            mesh.FaceNormals.ComputeFaceNormals();

            GeodesicsFromLevelSets levelSets = new GeodesicsFromLevelSets(mesh, initialValues, desiredWidth, lambda, nu);

            var optimizer = new Bobyqa(mesh.Vertices.Count, levelSets.Compute);
            optimizer.MaximumFunctionCalls = maxCalls;
            double[] x = initialValues.ToArray();


            var func = new LevelSetOpt(levelSets);

            var opt = new clsOptNelderMead(func);
            //if (optim == 1) opt = new clsOptNelderMead(func);
            //if (optim == 2) opt = new clsOptSimulatedAnnealing(func);
            //if (optim == 3) opt = new clsFireFly(func);
            opt.IsUseCriterion = false;
            opt.InitialPosition = initialValues.ToArray();
            opt.Iteration = maxCalls;
            double finalError = 0;
            bool retry = false;
            do
            {
                double[] resultX = { };
                if (optim == 0)
                {
                    var result = optimizer.FindMinimum(x);
                    finalError = result.F;
                    resultX = result.X;
                }
                else if (optim > 0)
                { // Nelder mead & simulated annealing
                    opt.Init();

                    while (opt.DoIteration(100) == false)
                    {
                        var eval = opt.Result.Eval;

                        //my criterion
                        if (eval < 10)
                        {
                            break;
                        }
                        Console.WriteLine("Iter: {0} Eval: {1}", opt.IterationCount, opt.Result.Eval);
                    }
                    finalError = opt.Result.Eval;
                    clsUtil.DebugValue(opt);
                }

                //if (optim == 0) {
                //    String str = "Optimization ended with an error of" + finalError + "\nDo you wish to run again using this results as input?";
                //    if (MessageBox.Show(str, "Optimization Ended", MessageBoxButtons.YesNo, MessageBoxIcon.Stop, MessageBoxDefaultButton.Button1) == DialogResult.Yes)
                //    {
                //        x = resultX;
                //        retry = true;
                //    }
                //    else
                //    {
                //        retry = false;
                //    }
                //}
            } while (retry == true);

            //levelSets = func.levelSets;

            List<double> desiredLevels = new List<double>();
            double max = 0.0;
            double min = 0.0;
            for (int i = 0; i < levelSets.VertexValues.Count; i++)
            {
                double value = levelSets.VertexValues[i];
                if (i == 0) { max = value; min = value; }

                if (value < min) min = value;
                if (value > max) max = value;
            }

            double diff = max - min;
            int levelCount = (int)(diff / desiredWidth);

            for (int j = 0; j <= levelCount; j++) desiredLevels.Add(min + (j * desiredWidth));

            DataTree<Line> pattern = levelSets.DrawLevelSetCurves(desiredLevels);
            List<double> divergence = levelSets.ComputeDivergence(false);
            List<double> divergenceNorm = levelSets.ComputeDivergence(true);

            DA.SetDataTree(0, pattern);
            DA.SetDataList(1, levelSets.Gradient);
            DA.SetDataList(2, divergence);
            DA.SetDataList(3, levelSets.VertexVoronoiArea);
            DA.SetData(4, finalError);
            DA.SetDataList(5, levelSets.VertexValues);
            DA.SetDataList(6, divergenceNorm);

        }

        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Properties.Resources.LevelSetG;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("c9253440-065f-438e-9c54-1c45f82085f3"); }
        }
    }
}
