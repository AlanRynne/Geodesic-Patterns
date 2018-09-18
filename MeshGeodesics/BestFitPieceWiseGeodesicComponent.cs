using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

using Cureos.Numerics.Optimizers;

namespace MeshGeodesics
{
    /// <summary>
    /// Best fit piece wise geodesic component.
    /// </summary>
    public class BestFitPieceWiseGeodesicComponent : GH_Component
    {

        /// <summary>
        /// Initializes a new instance of the <see cref="T:MeshGeodesics.BestFitPieceWiseGeodesicComponent"/> class.
        /// </summary>
        public BestFitPieceWiseGeodesicComponent()
          : base("BF Piece-wise Geodesic", "BF PW-Geod",
            "Best fit piecewise geodesic",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers the input parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh to draw geodesics on", GH_ParamAccess.item);
            pManager.AddCurveParameter("Perp. Geodesics", "Pg", "Set of reference measurement geodesics on a given mesh.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Parameter at Pg", "t", "Parameter of point of reference on Pg", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Maximum Iterations", "Iter", "Integer representing the maximum number of steps for the geodesic curve algorithm", GH_ParamAccess.item, 5);
            pManager.AddBooleanParameter("Both Directions", "BothDir", "Generate the geodesic on both directions",GH_ParamAccess.item,false);
            pManager.AddIntegerParameter("Start Point Index", "StIndex", "Index of starting point to use from the t' list", GH_ParamAccess.item);
            pManager.AddNumberParameter("Threshold", "e", "Margin of error alowed for the selection of the interval.", GH_ParamAccess.item);
            pManager.AddCurveParameter("Start Geodesic", "Sg", "Starting curve for pattern", GH_ParamAccess.item);
            pManager.AddNumberParameter("Width", "w", "Desired pattern width", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Perp step size", "Ps", "Separation between perp geodesics", GH_ParamAccess.item, 0.5);

        }

        /// <summary>
        /// Registers the output parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Best Fit Geodesic", "BF Geod", "Best fitting geodesic", GH_ParamAccess.list);
            pManager.AddNumberParameter("Best fit result", "R", "Best fit result", GH_ParamAccess.list);
            pManager.AddCurveParameter("PerpG", "Pgs", "Perp g's of last step", GH_ParamAccess.tree);

        }

        /// <summary>
        /// Solves the instance.
        /// </summary>
        /// <param name="DA">Da.</param>
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
            Curve initialCurve = null;
            double specifiedLength = 0.0;
            double perpStepSize = 0.3;

            // Set the input data
            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetDataList(1, perpGeodesics)) return;
            if (!DA.GetDataList(2, perpParameters)) return;
            if (!DA.GetData(3, ref maxIter)) return;
            if (!DA.GetData(4, ref bothDir)) return;
            if (!DA.GetData(5, ref startIndex)) return;
            if (!DA.GetData(6, ref threshold)) return;
            if (!DA.GetData(7, ref initialCurve)) return;
            if (!DA.GetData(8, ref specifiedLength)) return;
            if (!DA.GetData(9, ref perpStepSize)) return;

            // Data validation
            if (maxIter == 0) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "MaxIter cannot be 0");
            if (!mesh.IsValid) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Mesh is invalid");


            DataTree<Curve> pattern = new DataTree<Curve>();
            Curve previousCurve = initialCurve;
            List<Curve> tempPerpGeods = new List<Curve>();
            List<double> tempPerpParams = new List<double>();
            for (int i = 0; i < maxIter; i++)
            {
                Debug.WriteLine("Iter " + i);
                //  Create placeholder lists
                tempPerpGeods = new List<Curve>();
                tempPerpParams = new List<double>();

                // Divide curve
                double[] geodParams = previousCurve.DivideByLength(perpStepSize, true);

                if (geodParams == null)
                {

                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "NO  points found on iter" + i);
                    break;

                }
                if (geodParams.Length <= 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Not enough points found on iter" + i);
                    break;
                }

                foreach (double t in geodParams)
                {
                    // Get curve tangent vector and mesh normal
                    Point3d point = previousCurve.PointAt(t);
                    Vector3d tangent = previousCurve.TangentAt(t);
                    Vector3d normal = mesh.NormalAt(mesh.ClosestMeshPoint(point, 0.0));

                    // Rotate vector against normals 90 degrees
                    tangent.Rotate(0.5 * Math.PI, normal);

                    // Generate perp geodesic
                    Curve perpGeodesic = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, point, tangent, 100).ToNurbsCurve();

                    // Check for success
                    if (perpGeodesic != null && perpGeodesic.GetLength() > specifiedLength)
                    {
                        // Divide by specified length
                        double perpT = 0.0;
                        perpGeodesic.LengthParameter(specifiedLength, out perpT);

                        // Add data to lists
                        tempPerpGeods.Add(perpGeodesic);
                        tempPerpParams.Add(perpT);
                    }
                }

                // Clean perp geods of intersections ocurring BEFORE the specified distance
                var result = CleanPerpGeodesics(tempPerpGeods, tempPerpParams, specifiedLength);
                tempPerpGeods = result.perpGeodesics;
                tempPerpParams = result.perpParams;

                if (tempPerpGeods.Count < 3)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Not  enough perp geodesics where found for iter " + i);
                    break;
                }
                else
                {
                    //Generate the next piecewise geodesic
                    List<Curve> iterCurves = GeneratePiecewiseGeodesicCurve(mesh, tempPerpParams, tempPerpGeods, 1000, bothDir, 1, threshold);
                    // Add it to the pattern
                    pattern.AddRange(iterCurves, new GH_Path(i));
                    // Assign as previous for the next round
                    Curve[] joinedResult = Curve.JoinCurves(iterCurves);
                    if (joinedResult.Length > 1)
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "More than 1 curve after Join in iter " + i);
                        break;
                    }
                    previousCurve = Curve.JoinCurves(iterCurves)[0];
                }

            }


            DA.SetDataTree(0, pattern);
            DA.SetDataList(2, tempPerpGeods);

        }

        /// <summary>
        /// Gets the icon.
        /// </summary>
        /// <value>The icon.</value>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.PieceWiseG;
            }
        }

        /// <summary>
        /// Gets the component GUID.
        /// </summary>
        /// <value>The component GUID.</value>
        public override Guid ComponentGuid
        {
            get { return new Guid("8d62dd90-22c3-4cf3-9d37-3d4feb22d631"); }
        }

        public List<Curve> GeneratePiecewiseGeodesicCurve(Mesh mesh, List<double>  perpParameters, List<Curve> perpGeodesics, int maxIter, bool bothDir, int startIndex, double threshold)
        {
            // Generate Initial values and variable Bounds for the optimization problem.
            // Only using first variable for now, the extra variable is just to make it work.
            Random rnd = new Random();
            double limit = 0.05;
            double start = (rnd.NextDouble() * limit) - (limit / 2);
            double[] startData = { start, 0 };
            double[] xl = new double[] { -limit * Math.PI, -1 };
            double[] xu = new double[] { limit * Math.PI, 1 };


            BestFitPieceWiseGeodesic bestFitG = new BestFitPieceWiseGeodesic(mesh, perpGeodesics, perpParameters, maxIter, bothDir, startIndex, threshold, Vector3d.Unset);

            var optimizer = new Bobyqa(2, bestFitG.ComputeError, xl, xu);
            var result = optimizer.FindMinimum(startData);

            if (bestFitG.bestFitInterval.Length == 0)
            {
                do
                {
                    limit += 0.01;
                    start = (rnd.NextDouble() * limit) - (limit / 2);
                    startData[0] = start;
                    xl = new double[] { -limit * Math.PI, -1 };
                    xu = new double[] { limit * Math.PI, 1 };
                    optimizer = new Bobyqa(2, bestFitG.ComputeError, xl, xu);
                    result = optimizer.FindMinimum(startData);
                } while (limit < 0.35);

                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No best interval found?!");

            }

            // Sub curves methods go here
            List<Curve> pieceWiseList = new List<Curve>();
            pieceWiseList = bestFitG.GenerateSubCurves(startData, xl, xu, 0);
            return pieceWiseList;
        }

        public struct CleanPerps
        {
            public List<Curve> perpGeodesics;
            public List<double> perpParams;
        }

        public CleanPerps CleanPerpGeodesics(List<Curve> perpGeodesics, List<double> perpParams, double distance)
        {
            List<Curve> cleanCurves = perpGeodesics;
            List<double> cleanParams = perpParams;
            double specifiedDistance = distance;

            bool erased = false;
            do
            {
                erased = false;
                for (int i = (cleanCurves.Count - 1); i >= 1; i--)
                {
                    Curve thisCurve = cleanCurves[i];
                    Curve nextCurve = cleanCurves[i - 1];
                    var crvInt = Rhino.Geometry.Intersect.Intersection.CurveCurve(thisCurve, nextCurve, 0.0, 0.0);
                    if (crvInt.Count != 0 && crvInt != null)
                    {
                        Interval dom = new Interval(0, crvInt[0].ParameterA);
                        double dist = thisCurve.GetLength(dom);
                        if (dist < specifiedDistance)
                        {
                            cleanCurves.RemoveAt(i);
                            cleanParams.RemoveAt(i);
                            erased = true;
                        }
                    }
                }

            } while (erased == true);

            CleanPerps result = new CleanPerps();
            result.perpGeodesics = cleanCurves;
            result.perpParams = cleanParams;

            return result;
        }
    }
}
