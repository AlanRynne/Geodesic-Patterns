using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

using Cureos.Numerics.Optimizers;

namespace MeshGeodesics
{
    public class BestFitPieceWiseGeodesicComponent : GH_Component
    {
        public BestFitPieceWiseGeodesicComponent()
          : base("BF Piece-wise Geodesic", "BF PW-Geod",
            "Best fit piecewise geodesic",
            "Alan", "Geodesic Patterns")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh to draw geodesics on", GH_ParamAccess.item);
            pManager.AddCurveParameter("Perp. Geodesics", "Pg", "Set of reference measurement geodesics on a given mesh.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Parameter at Pg", "t", "Parameter of point of reference on Pg", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Maximum Iterations", "Iter", "Integer representing the maximum number of steps for the geodesic curve algorithm", GH_ParamAccess.item, 50);
            pManager.AddBooleanParameter("Both Directions", "BothDir", "Generate the geodesic on both directions",GH_ParamAccess.item,false);
            pManager.AddIntegerParameter("Start Point Index", "StIndex", "Index of starting point to use from the t' list", GH_ParamAccess.item);
            pManager.AddNumberParameter("Threshold", "e", "Margin of error alowed for the selection of the interval.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Best Fit Geodesic", "BF Geod", "Best fitting geodesic", GH_ParamAccess.list);
            pManager.AddNumberParameter("Best fit result", "R", "Best fit result", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Properties
            Mesh mesh = null;
            List<Curve> perpGeodesics = new List<Curve>();
            List<double> perpParameters = new List<double>();
            int maxIter = 0;
            bool bothDir = false;
            int startIndex = 0;
            mesh = null;
            perpGeodesics = new List<Curve>();
            perpParameters = new List<double>();
            double threshold = 0.0;

            // Set the input data
            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetDataList(1, perpGeodesics)) return;
            if (!DA.GetDataList(2, perpParameters)) return;
            if (!DA.GetData(3, ref maxIter)) return;
            if (!DA.GetData(4, ref bothDir)) return;
            if (!DA.GetData(5, ref startIndex)) return;
            if (!DA.GetData(6, ref threshold)) return;

            // Data validation
            if (maxIter == 0) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "MaxIter cannot be 0");
            if (!mesh.IsValid) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Mesh is invalid");
            if (perpGeodesics.Count < 2) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "There must be at least 2 perpendicular geodesics");
            if (perpParameters.Count != perpGeodesics.Count) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Curves and Params list must have the same number of items");

            // Generate Initial values and variable Bounds for the optimization problem.
            // Only using first variable for now, the extra variable is just to make it work.
            double[] startData = { 0.010, 0 };
            double[] xl = new double[] { -0.15, -1 };
            double[] xu = new double[] { 0.15, 1 };


            BestFitPieceWiseGeodesic bestFitG = new BestFitPieceWiseGeodesic(mesh, perpGeodesics, perpParameters, maxIter, bothDir, startIndex, threshold, Vector3d.Unset);

            var optimizer = new Bobyqa(2, bestFitG.ComputeError, xl, xu);
            var result = optimizer.FindMinimum(startData);

            if (bestFitG.bestFitInterval.Length == 0) {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No best interval found?!");
                return;
            }

            // Sub curves methods go here
            List<Curve> pieceWiseList = new List<Curve>();
            pieceWiseList = bestFitG.GenerateSubCurves(startData,xl,xu);


            DA.SetDataList(0, pieceWiseList);
            DA.SetDataList(1, result.X);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.PieceWiseG;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("8d62dd90-22c3-4cf3-9d37-3d4feb22d631"); }
        }

    }
}
