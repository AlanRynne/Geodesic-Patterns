using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Collections;

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
            pManager.AddCurveParameter("Start Geodesic", "Sg", "Starting curve for pattern", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Both Directions", "BothDir", "Generate the geodesic on both directions", GH_ParamAccess.item, false);
            pManager.AddIntegerParameter("Max. Pattern Count", "Pcount", "Maximum number of curves to be generated", GH_ParamAccess.item, 5);
            pManager.AddNumberParameter("Threshold", "e", "Margin of error alowed for the selection of the interval.", GH_ParamAccess.item, 0.07);
            pManager.AddNumberParameter("Width", "w", "Desired pattern width", GH_ParamAccess.item, 1);
            pManager.AddNumberParameter("Perp step size", "Ps", "Separation between perp geodesics", GH_ParamAccess.item, 0.5);
            pManager.AddNumberParameter("Minimization threshold", "minTh", "Treshold for best-fit geodesic minimization", GH_ParamAccess.item, 0.1);
        }

        /// <summary>
        /// Registers the output parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Best Fit Geodesic", "BF Geod", "Best fitting geodesic", GH_ParamAccess.list);
            pManager.AddCurveParameter("Split Lines", "sL", "Lines on the piecewise geodesics breakpoints", GH_ParamAccess.list);
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
            Curve initialCurve = null;
            double specifiedDistance = 0.0;
            int maxCount = 0;
            double threshold = 0.0;
            double perpStepSize = 0.0;
            bool bothDir = false;
            double minThreshold = 0.0;

            // Set the input data
            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetData(1, ref initialCurve)) return;
            if (!DA.GetData(2, ref bothDir)) return;
            if (!DA.GetData(3, ref maxCount)) return;
            if (!DA.GetData(4, ref threshold)) return;
            if (!DA.GetData(5, ref specifiedDistance)) return;
            if (!DA.GetData(6, ref perpStepSize)) return;
            if (!DA.GetData(7, ref minThreshold)) return;

            // Data validation
            if (maxCount == 0) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Count cannot be 0");
            if (!mesh.IsValid) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Mesh is invalid");

            // Placeholder properties
            DataTree<Curve> pattern = new DataTree<Curve>();
            Curve previousCurve = initialCurve;
            List<Curve> perpGeods = new List<Curve>();
            List<double> perpParams = new List<double>();

            // Start piecewise evolution process
            for (int i = 0; i < maxCount; i++)
            {
                Debug.WriteLine("Iter " + i);
                //  Create placeholder lists
                perpGeods = new List<Curve>();
                perpParams = new List<double>();

                // Divide curve
                double[] geodParams = previousCurve.DivideByLength(perpStepSize, true);

                if (geodParams == null)
                {

                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No  points found on iter" + i);
                    break;

                }
                if (geodParams.Length <= 2)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Not enough points found on iter" + i);
                    break;
                }

                // Generate perp geodesics for measurement
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
                    if (perpGeodesic != null && perpGeodesic.GetLength() > specifiedDistance)
                    {
                        // Divide by specified length
                        double perpT = 0.0;
                        perpGeodesic.LengthParameter(specifiedDistance, out perpT);

                        // Add data to lists
                        perpGeods.Add(perpGeodesic);
                        perpParams.Add(perpT);
                    }
                }
                // Clean perp geods of intersections ocurring BEFORE the specified distance
                var result = CleanPerpGeodesics(perpGeods, perpParams, specifiedDistance);
                // Assign clean lists
                perpGeods = result.perpGeodesics;
                perpParams = result.perpParams;


                // Break if not enough perpGeods remain
                if (perpGeods.Count < 6)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Not  enough perp geodesics where found for iter " + i);
                    break;
                }

                //Generate the next piecewise geodesic
                List<Curve> iterCurves = GeneratePiecewiseGeodesicCurve(mesh, perpParams, perpGeods, 1000, bothDir, 0, threshold, Vector3d.Unset);

                // Add it to the pattern
                pattern.AddRange(iterCurves, new GH_Path(i));

                // Assign as previous for the next round
                Curve[] joinedResult = Curve.JoinCurves(iterCurves);

                // Error check
                if (joinedResult.Length > 1)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "More than 1 curve after Join in iter " + i);
                    break;
                }
                //Create points and bisectrix vectors for next round of perp geodesics
                Point3dList ptList = new Point3dList();
                foreach (Curve c in iterCurves)
                {
                    if (!ptList.Contains(c.PointAtStart)) ptList.Add(c.PointAtStart);
                    if (!ptList.Contains(c.PointAtEnd)) ptList.Add(c.PointAtEnd);

                }
                Debug.WriteLine("ptList Count: " + ptList.Count);
                // Assign new curve to previous curve
                Curve joinedCurve = joinedResult[0];
                previousCurve = joinedCurve;
            }

            // Assign data to output
            DA.SetDataTree(0, pattern);
            DA.SetDataList(2, perpGeods);

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

        public List<Curve> GeneratePiecewiseGeodesicCurve(Mesh mesh, List<double>  perpParameters, List<Curve> perpGeodesics, int maxIter, bool bothDir, int startIndex, double threshold, Vector3d dir)
        {
            // Generate Initial values and variable Bounds for the optimization problem.
            // Only using first variable for now, the extra variable is just to make it work.
            Random rnd = new Random();
            double limit = 0.35;
            double start = (rnd.NextDouble() * limit) - (limit / 2);

            double[] startData = { start, 0 };
            double[] xl = new double[] { -limit * Math.PI, -1 };
            double[] xu = new double[] { limit * Math.PI, 1 };

            // Generate bestfit G
            BestFitPieceWiseGeodesic bestFitG = new BestFitPieceWiseGeodesic(mesh, perpGeodesics, perpParameters, maxIter, bothDir, startIndex, threshold, dir);

            // Run optimization
            var optimizer = new Bobyqa(2, bestFitG.ComputeError, xl, xu);
            var result = optimizer.FindMinimum(startData);

            // If no best fit is found, gradually increase angle
            if (bestFitG.bestFitInterval.Length == 0 || result.F > 0.1)
            {
                do
                {
                    limit += 0.05;
                    Debug.WriteLine("Limit increased :" + limit);
                    start = (rnd.NextDouble() * limit) - (limit / 2);
                    startData[0] = start;
                    xl = new double[] { -limit * Math.PI, -1 };
                    xu = new double[] { limit * Math.PI, 1 };
                    optimizer = new Bobyqa(2, bestFitG.ComputeError, xl, xu);
                    result = optimizer.FindMinimum(startData);
                    //Debug.WriteLine("Result: " + result.F + " Interval: " + bestFitG.bestFitInterval[0] + "-" + bestFitG.bestFitInterval[1]);
                } while (limit < 0.35);

                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No best interval found?!");
                if (bestFitG.bestFitInterval.Length == 0) return new List<Curve>(){bestFitG.selectedGeodesic};

            }
            //Debug.WriteLine("Result: " + result.F + " Interval: " + bestFitG.bestFitInterval[0] + "-" + bestFitG.bestFitInterval[1]);

            // Split the curve
            double domainLength = bestFitG.bestFitInterval[1] - bestFitG.bestFitInterval[0];
            Curve splitC = bestFitG.selectedGeodesic;
            splitC.Domain = new Interval(0, 1);
            List<Curve> pieceWiseList = new List<Curve>();
            if (domainLength < perpGeodesics.Count-4)
            {
                
                // Generate piecewise at end
                if (bestFitG.bestFitInterval[1] < perpGeodesics.Count - 3)
                {
                    // Get reference point for splitting
                    //var refInt = Intersection.CurveCurve(splitC, perpGeodesics[(bestFitG.bestFitInterval[1] - 1)], 0.0, 0.0);
                    //double refT = refInt[0].ParameterA;

                    // Get intersection with end point
                    var endInt = Intersection.CurveCurve(splitC, perpGeodesics[(bestFitG.bestFitInterval[1])], 0.0, 0.0);
                    var endParam = endInt[0].ParameterA;

                    Point3d refPoint = splitC.PointAt(endParam / 2);
                        
                    // Split the curve
                    Curve[] splitGs = splitC.Split(endParam);
                    // Select the correct curve
                    foreach (Curve g in splitGs)
                    {
                        double t;
                        g.ClosestPoint(refPoint, out t);
                        double distance = refPoint.DistanceTo(g.PointAt(t));
                        if (distance < 0.01)
                        {
                            splitC = g;
                            break;
                        }
                    }

                    List<double> paramsAtEnd = new List<double>(perpParameters);
                    paramsAtEnd.RemoveRange(0, bestFitG.bestFitInterval[1]);
                    List<Curve> geodsAtEnd = new List<Curve>(perpGeodesics);
                    geodsAtEnd.RemoveRange(0, bestFitG.bestFitInterval[1]);
                    // Replace first parameter with the parameter of the piecewise curve endpoint at the perpGeod.
                    var events = Intersection.CurveCurve(geodsAtEnd[0], bestFitG.selectedGeodesic, 0.0, 0.0);
                    paramsAtEnd[0] = events[0].ParameterA;
                    // Get the best next best fit geodesic at end
                    pieceWiseList.AddRange(GeneratePiecewiseAtEnd(mesh, paramsAtEnd, geodsAtEnd, maxIter, 0, threshold));
                }

                //// Generate piecewise at start
                //if (bestFitG.bestFitInterval[0] > 3)
                //{
                //    var refInt = Intersection.CurveCurve(splitC, perpGeodesics[(bestFitG.bestFitInterval[0] + 2)], 0.0, 0.0);
                //    Point3d refPoint = refInt[0].PointB;
                //    // Get intersection with end point
                //    var startInt = Intersection.CurveCurve(splitC, perpGeodesics[(bestFitG.bestFitInterval[0])], 0.0, 0.0);
                //    var startParam = startInt[0].ParameterA;
                //    // Split the curve
                //    Curve[] splitGs = splitC.Split(startParam);
                //    // Select the correct curve
                //    foreach (Curve g in splitGs)
                //    {
                //        double t;
                //        g.ClosestPoint(refPoint, out t);
                //        double distance = refPoint.DistanceTo(g.PointAt(t));
                //        if (distance < 0.01)
                //        {
                //            splitC = g;
                //            break;
                //        }
                //    }

                //    List<double> paramsAtStart = new List<double>(perpParameters);
                //    paramsAtStart.RemoveRange(bestFitG.bestFitInterval[0], (perpParameters.Count - 1) - bestFitG.bestFitInterval[0] - 1);
                //    List<Curve> geodsAtStart = new List<Curve>(perpGeodesics);
                //    geodsAtStart.RemoveRange(bestFitG.bestFitInterval[0], (perpGeodesics.Count - 1) - bestFitG.bestFitInterval[0] - 1);

                //    // Replace first parameter with the parameter of the piecewise curve endpoint at the perpGeod.
                //    paramsAtStart.Reverse();
                //    geodsAtStart.Reverse();
                //    var events = Intersection.CurveCurve(geodsAtStart[0], bestFitG.selectedGeodesic, 0.0, 0.0);
                //    paramsAtStart[0] = events[0].ParameterA;
                //    Vector3d vector = -splitC.TangentAtStart;
                //    // This would be the same as at end, but with a first step flipping the direction of the perpGeodesics list.
                //    /pieceWiseList.AddRange(GeneratePiecewiseAtStart(mesh, paramsAtStart, geodsAtStart, maxIter, 0, threshold, vector));
                //}

                pieceWiseList.Add(splitC);

            }
            else
            {
                pieceWiseList.Add(bestFitG.selectedGeodesic);
            }

            //pieceWiseList = bestFitG.GenerateSubCurves(startData, xl, xu, 0);
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
            double specifiedDistance = distance * 1.5;
            bool reversed = false;
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
                cleanCurves.Reverse();
                perpParams.Reverse();
                if (reversed) reversed = false;
                else reversed = true;


            } while (erased == true);

            if (reversed)
            {
                cleanCurves.Reverse();
                perpParams.Reverse();
            }

            CleanPerps result = new CleanPerps();
            result.perpGeodesics = cleanCurves;
            result.perpParams = cleanParams;

            return result;
        }

        public Curve SplitGeodesicWithCurves(Curve geod, Curve perp1, Curve perp2)
        {
            Curve tempG = geod.DuplicateCurve(); // Copy curve to temp
            tempG.Domain = new Interval(0, 1); // Normalize curve domain

            // Get intersections
            CurveIntersections curveInt = Intersection.CurveCurve(tempG, perp1, 0.0001, 0.0001);
            CurveIntersections curveInt2 = Intersection.CurveCurve(tempG, perp2, 0.0001, 0.0001);

            double t1 = tempG.Domain.T0;
            double t2 = tempG.Domain.T1;

            if (curveInt.Count != 0) t1 = curveInt[0].ParameterA;
            if (curveInt2.Count != 0) t2 = curveInt2[0].ParameterA;

            double tMid;
            if (t1 > t2)
            {
                double temp = t1;
                t1 = t2;
                t2 = temp;
            }
            tMid = (t2 + t1) / 2;

            Point3d midPoint = tempG.PointAt(tMid);

            if (curveInt.Count != 0 && t1 > tempG.Domain.T0)
            {

                Curve[] splitCurves = tempG.Split(t1);
                if (splitCurves != null)
                {
                    foreach (Curve crv in splitCurves)
                    {
                        double closestT;
                        crv.ClosestPoint(midPoint, out closestT);

                        double distance = midPoint.DistanceTo(crv.PointAt(closestT));
                        if (distance < 0.001)
                        {
                            tempG = crv;
                        }
                    }
                }

            }

            tempG.Domain = new Interval(0, 1);
            t2 = tempG.Domain.T1;
            curveInt2 = Intersection.CurveCurve(tempG, perp2, 0.0001, 0.0001);
            if (curveInt2.Count != 0) t2 = curveInt2[0].ParameterA;

            if (curveInt2.Count != 0 && t2 < tempG.Domain.T1 && t2 > 0)
            {
                Curve[] splitCurves = tempG.Split(t2);
                if (splitCurves != null)
                {
                    foreach (Curve crv in splitCurves)
                    {
                        double closestT;
                        crv.ClosestPoint(midPoint, out closestT);

                        double distance = midPoint.DistanceTo(crv.PointAt(closestT));
                        if (distance < 0.001)
                        {
                            tempG = crv;
                        }
                    }
                }

            }


            return tempG;
        }
    
        public List<Curve> GeneratePiecewiseAtEnd(Mesh mesh, List<double> perpParameters, List<Curve> perpGeodesics, int maxIter, int startIndex, double threshold)
        {
            Debug.WriteLine("Piecewise at END called");
            return GeneratePiecewiseGeodesicCurve(mesh, perpParameters, perpGeodesics, maxIter, false, startIndex, threshold, Vector3d.Unset);
        }

        public List<Curve> GeneratePiecewiseAtStart(Mesh mesh, List<double> perpParameters, List<Curve> perpGeodesics, int maxIter, int startIndex, double threshold, Vector3d dir)
        {
            Debug.WriteLine("Piecewise at START called");
            // Must invert direction of twist vector...
            return GeneratePiecewiseGeodesicCurve(mesh, perpParameters, perpGeodesics, maxIter, false, startIndex, threshold, dir);
        }
    }
}
