using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

using Cureos.Numerics.Optimizers;

namespace MeshGeodesics
{

    /// <summary>
    /// Mesh geodesic methods.
    /// </summary>
    public static class MeshGeodesicMethods
    {

        /// <summary>
        /// Gets the geodesic curve on mesh.
        /// </summary>
        /// <returns>The geodesic curve on mesh.</returns>
        /// <param name="mesh">Mesh.</param>
        /// <param name="startPoint">Start point.</param>
        /// <param name="startDirection">Start direction.</param>
        /// <param name="iter">Iter.</param>
        public static Polyline getGeodesicCurveOnMesh(Mesh mesh, Point3d startPoint, Vector3d startDirection, int iter)
        {
            double tol = 0.001;
            // Get meshPoint from start location
            mesh.UnifyNormals();
            mesh.Normals.ComputeNormals();
            MeshPoint mP = mesh.ClosestMeshPoint(startPoint, 0.0);
            int thisFaceIndex = mP.FaceIndex;
            //Debug.WriteLine("Face index:{0}", mP.FaceIndex);
            int thisEdgeIndex = -1;
            // Project direction to be tangent to the mesh.
            Vector3d norm = mesh.FaceNormals[thisFaceIndex];
            Vector3d crossP = Vector3d.CrossProduct(startDirection, norm);
            // FIXME: Currently not tangent :S

            Vector3d correctedDir = Vector3d.CrossProduct(norm, crossP);
            correctedDir.Unitize();

            // Empty lists for result values
            List<Point3d> geodesicPoints = new List<Point3d>();
            List<Vector3d> geodesicVectors = new List<Vector3d>();
            // Add initial values to the lists
            geodesicPoints.Add(startPoint);
            geodesicVectors.Add(correctedDir);

            int nextFaceIndex = -1;
            for (int i = 0; i < iter; i++)
            {
                //Debug.WriteLine("Iter {0}", i);
                if (nextFaceIndex == -2) break;

                // Get starting meshFace
                MeshFace thisFace = mesh.Faces[thisFaceIndex];
                int[] faceEdges = mesh.TopologyEdges.GetEdgesForFace(thisFaceIndex);

                Plane directionPlane = new Plane(mP.Point, mesh.NormalAt(mP), correctedDir);

                foreach (int edgeIndex in faceEdges)
                {
                    //Debug.WriteLine("new edge {0}", edgeIndex);
                    if (edgeIndex == thisEdgeIndex) continue;
                    Line edgeLine = mesh.TopologyEdges.EdgeLine(edgeIndex);
                    double t;
                    bool success = Rhino.Geometry.Intersect.Intersection.LinePlane(edgeLine, directionPlane, out t);
                    if (success && t > 0 && t < 1)
                    {
                        //Debug.WriteLine("t: {0}, edge-t0: {1}, edge-t1:{2} ", t, edgeLine.ClosestParameter(edgeLine.From), edgeLine.ClosestParameter(edgeLine.To));
                        // Check if the the point is IN FRONT or BEHIND the current direction.
                        Vector3d newV = new Vector3d(edgeLine.PointAt(t) - mP.Point);
                        newV.Unitize();
                        double angle = Vector3d.VectorAngle(correctedDir, newV);
                        //Debug.WriteLine("Vector angle: {0}", angle);
                        // Only continue if angle is THE SAME (considered as a threshold for consistency)
                        if (angle > 0.05) continue;

                        //Debug.WriteLine("Continue with this point");

                        Point3d nextPoint = edgeLine.PointAt(t);
                        int[] connectedFaces = mesh.TopologyEdges.GetConnectedFaces(edgeIndex);
                        //Debug.WriteLine("ConnectedFaces count: {0}", connectedFaces.Length);
                        if (connectedFaces.Length == 1)
                        {
                            nextFaceIndex = -2;
                            geodesicPoints.Add(nextPoint);
                            break;

                        }
                        foreach (int faceIndex in connectedFaces)
                        {
                            //Check which is NOT the current face
                            if (faceIndex != thisFaceIndex)
                            {
                                nextFaceIndex = faceIndex;
                            }
                        }
                        // If no adjacent face was found, the curve has reached the boundary.
                        if (nextFaceIndex == -1) { break; }
                        else
                        {
                            //Debug.WriteLine("Update data for next iter");
                            Vector3d faceNormal = mesh.FaceNormals[thisFaceIndex];
                            Vector3d nextFaceNormal = mesh.FaceNormals[nextFaceIndex];
                            Vector3d cP = Vector3d.CrossProduct(correctedDir, faceNormal);

                            correctedDir = Vector3d.CrossProduct(nextFaceNormal, cP);
                            correctedDir.Unitize();
                            mP = mesh.ClosestMeshPoint(nextPoint, tol);
                            geodesicPoints.Add(mP.Point); //Add new point to list;
                            geodesicVectors.Add(correctedDir); // Add corrected direction to list
                            thisFaceIndex = nextFaceIndex;
                            thisEdgeIndex = edgeIndex;
                            break;
                        }
                    }
                    else
                    {
                        //Debug.WriteLine("No intersection found");
                    }
                }

            }
            return new Polyline(geodesicPoints);
        }

        /// <summary>
        /// Computes the geodesic pattern by parralel transport.
        /// </summary>
        /// <returns>The geodesic pattern by parralel transport.</returns>
        /// <param name="mesh">Mesh.</param>
        /// <param name="startGeodesic">Start geodesic.</param>
        /// <param name="count">Count.</param>
        /// <param name="alpha">Alpha.</param>
        /// <param name="iter">Iter.</param>
        public static List<Curve> ComputeGeodesicPatternByParralelTransport(Mesh mesh, Curve startGeodesic, int count, double alpha, int iter)
        {
            //Sample curves using provided 'count'
            double[] parameters = startGeodesic.DivideByCount(count, true);

            //At each sample point obtain the mesh normal and the curve tangent
            List<Point3d> samplePoints = new List<Point3d>();
            List<Vector3d> tangentVectors = new List<Vector3d>();
            List<Vector3d> normalVectors = new List<Vector3d>();
            foreach (double t in parameters)
            {
                Point3d pt = startGeodesic.PointAt(t);
                Vector3d tang = startGeodesic.TangentAt(t);
                Vector3d normal = mesh.NormalAt(mesh.ClosestMeshPoint(pt, 0.0));
                samplePoints.Add(pt);
                tangentVectors.Add(tang);
                normalVectors.Add(normal);
            }

            //At the starting point of the curve: obtain the cross product of tangent and normal.
            Vector3d CP = Vector3d.CrossProduct(tangentVectors[0], normalVectors[0]);

            //Rotate the vector around the normal using the specified 'alpha' angle in radians
            CP.Rotate(alpha, normalVectors[0]);

            //Parallel transport the vector along the rest of the sample parameters.
            List<Vector3d> transportedVectors = VectorParallelTransport(CP, samplePoints, mesh);

            //Generate geodesics using the sample points and the transported vectors.
            List<Curve> geodesicPattern = new List<Curve>();
            for (int i = 0; i < samplePoints.Count; i++)
            {
                Curve geod = getGeodesicCurveOnMesh(mesh, samplePoints[i], transportedVectors[i], iter).ToNurbsCurve();
                Curve geodInverse = getGeodesicCurveOnMesh(mesh, samplePoints[i], -transportedVectors[i], iter).ToNurbsCurve();
                Curve fullGeodesic = Curve.JoinCurves(new List<Curve> { geod, geodInverse })[0];
                geodesicPattern.Add(fullGeodesic);
            }

            //Return geodesic patterns 
            return geodesicPattern;
        }

        /// <summary>
        /// Parallel transport frame.
        /// </summary>
        public struct ParallelTransportFrame
        {
            public Vector3d T;
            public Vector3d N;
            public Vector3d B;
            public Point3d position;
        }

        /// <summary>
        /// Move vector along frames using parallel transport
        /// </summary>
        /// <returns>The parallel transport frames.</returns>
        /// <param name="V">V.</param>
        /// <param name="pts">Pts.</param>
        public static List<ParallelTransportFrame> ParallelTransportFrames(Vector3d V, List<Point3d> pts)
        {
            Vector3d V_prev = V;
            List<ParallelTransportFrame> frameList = new List<ParallelTransportFrame>();
            for (int i = 0; i < pts.Count - 2; i++)
            {
                Vector3d t0 = new Vector3d(pts[i + 1] - pts[i]);
                Vector3d t1 = new Vector3d(pts[i + 2] - pts[i + 1]);
                t0.Unitize();
                t1.Unitize();

                Vector3d B = Vector3d.CrossProduct(t0, t1);
                if (B.Length <= 0.01)
                {
                    V = V_prev;
                }
                else
                {
                    B.Unitize();
                    double theta = Math.Acos(Vector3d.Multiply(t0, t1));
                    V.Rotate(theta, B);
                }
                Vector3d Binorm = Vector3d.CrossProduct(t0, V);
                Binorm.Unitize();

                ParallelTransportFrame frame = new ParallelTransportFrame();
                frame.T = t0;
                frame.N = V;
                frame.B = Binorm;
                frame.position = pts[i];
                frameList.Add(frame);
                // push back frame?
                V_prev = V;
            }

            return frameList;
        }

        /// <summary>
        /// Vectors the parallel transport.
        /// </summary>
        /// <returns>The parallel transport.</returns>
        /// <param name="vector">Vector.</param>
        /// <param name="points">Points.</param>
        /// <param name="mesh">Mesh.</param>
        public static List<Vector3d> VectorParallelTransport(Vector3d vector, List<Point3d> points, Mesh mesh)
        {
            List<Vector3d> transportedVectors = new List<Vector3d>();
            transportedVectors.Add(vector);

            for (int i = 1; i < points.Count; i++)
            {
                Vector3d CP = Vector3d.CrossProduct(transportedVectors[i - 1], mesh.NormalAt(mesh.ClosestMeshPoint(points[i - 1], 0.0)));
                Vector3d newV = Vector3d.CrossProduct(mesh.NormalAt(mesh.ClosestMeshPoint(points[i - 1], 0.0)), CP);
                transportedVectors.Add(newV);
            }

            return transportedVectors;



        }

    }

    /// <summary>
    /// Best fit piece-wise geodesic.
    /// </summary>
    public class BestFitPieceWiseGeodesic: BestFitGeodesic
    {
        // Properties
        public int[] bestFitInterval;
        readonly double threshold;
        Vector3d refDir;

        // Constructor
        public BestFitPieceWiseGeodesic(Mesh aMesh, List<Curve> pGeods, List<double> pParams, int mIter, bool bDir, int stIndex, double thrshld, Vector3d _refDir)
            : base(aMesh, pGeods, pParams, mIter, bDir, stIndex)
        {
            threshold = thrshld;
            bestFitInterval = new int[]{};
            refDir = _refDir;

        }

        /// <summary>
        /// Computes the error.
        /// </summary>
        /// <returns>The error.</returns>
        /// <param name="n">Number of variables</param>
        /// <param name="x">Variable Initial Values</param>
        public new double ComputeError(int n, double[] x)
        {
            double alpha = x[0];
            bestFitInterval = new int[] { };
            Curve curve = perpGeodesics[startIndex];
            Point3d pt = curve.PointAt(perpParameters[startIndex]);
            Vector3d vector = curve.TangentAt(perpParameters[startIndex]);

            MeshPoint mPt = mesh.ClosestMeshPoint(pt, 0.0);
            Vector3d normal = mesh.NormalAt(mPt);
            Vector3d cP = Vector3d.CrossProduct(vector, normal);
            cP.Rotate(alpha, normal);

            if (refDir == Vector3d.Unset) refDir = cP;
            double angle = Vector3d.VectorAngle(cP, refDir);
            if (angle >= 0.1*Math.PI) cP = -cP;

            Vector3d.VectorAngle(cP, refDir);
            Curve newG = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, cP, maxIter).ToNurbsCurve();
            if (bothDir)
            {
                Curve newG2 = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, -cP, maxIter).ToNurbsCurve();
                newG = Curve.JoinCurves(new List<Curve> { newG, newG2 })[0];
            }

            // Assign resulting geodesic to global property for output.
            selectedGeodesic = newG;

            // Calculate error
            double error = 0;
            List<double> distances = new List<double>();
            List<double> signedDistances = new List<double>();

            for (int i = 0; i < perpGeodesics.Count - 1; i++)
            {
                Curve g = perpGeodesics[i];
                CurveIntersections cvInt = Intersection.CurveCurve(newG, g, 0.00001, 0.00001);
                double signedDistance = g.GetLength(new Interval(0, perpParameters[i]));
                signedDistances.Add(signedDistance);
                if (cvInt.Count > 0)
                {
                    // Compute distance if intersection is found
                    double param = cvInt[0].ParameterB;
                    double distance = g.GetLength(new Interval(0, param));
                    distances.Add(distance);
                    // Add squared distance to error
                    error += Math.Pow(signedDistance - distance, 2);
                }
                else
                {
                    // Penalize if no intersection is found on this perp geodesic
                    distances.Add(1000);
                    error += 1000;
                }
            }

            //Calculate longest interval within threshold.
            for (int k = (distances.Count-1); k >= 2 ;k--)
            {
                for (int i = 0; i < (distances.Count - k); i++)
                {
                    //Check if interval i->k is within bounds
                    bool flag = true;
                    for (int j = i; j < i+k; j++)
                    {
                        double Lbound = signedDistances[j] * (1 - threshold);
                        double Ubound = signedDistances[j] * (1 + threshold);
                        if (Lbound > distances[j] || distances[j] > Ubound)
                        {
                            flag = false;
                            break;
                        }
                    }
                    if (flag && bestFitInterval.Length == 0)
                    {
                        bestFitInterval = new int[]{i, i + k};
                    }
                }

            }
            if (bestFitInterval.Length == 0)
            {
                error += 1000000;
                return error;
            }
            error = error / (bestFitInterval[1] - bestFitInterval[0]);

            //if (invertDir) selectedGeodesic.Reverse();

            return error;
        }

        /// <summary>
        /// Cuts the curve between perp indexes.
        /// </summary>
        /// <returns>The curve between perp indexes.</returns>
        /// <param name="g">The green component.</param>
        /// <param name="perpGs">Perp gs.</param>
        /// <param name="index1">Index1.</param>
        /// <param name="index2">Index2.</param>
        public Curve CutCurveBetweenPerpIndexes(Curve g, List<Curve> perpGs, int index1, int index2)
        {
            Curve tempG = g.DuplicateCurve();

            tempG.Domain = new Interval(0, 1);

            CurveIntersections curveInt = Intersection.CurveCurve(tempG, perpGs[index1], 0.0001, 0.0001);
            CurveIntersections curveInt2 = Intersection.CurveCurve(tempG, perpGs[index2], 0.0001, 0.0001);

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

            tempG.Domain = new Interval(0, 1);
            t2 = tempG.Domain.T1;
            curveInt2 = Intersection.CurveCurve(tempG, perpGs[index2], 0.0001, 0.0001);
            if (curveInt2.Count != 0) t2 = curveInt2[0].ParameterA;

            if (curveInt2.Count != 0 && t2 < tempG.Domain.T1 && t2 > 0)
            {
                Curve[] splitCurves = tempG.Split(t2);
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


            return tempG;
        }
    
        /// <summary>
        /// Generates the sub curves.
        /// </summary>
        /// <returns>The sub curves.</returns>
        /// <param name="_startData">Start data.</param>
        /// <param name="_xl">Xl.</param>
        /// <param name="_xu">Xu.</param>
        public List<Curve> GenerateSubCurves(double[] _startData, double[] _xl, double[] _xu)
        {
            
            // Generate Initial values and variable Bounds for the optimization problem.
            // Only using first variable for now, the extra variable is just to make it work.
            double[] startData = _startData;
            double[] xl = _xl;
            double[] xu = _xu;

            List<Curve> pieceWiseList = new List<Curve>();
            pieceWiseList.Add(selectedGeodesic);

            if (bestFitInterval.Length != 0)
            {
                pieceWiseList[0] = CutCurveBetweenPerpIndexes(pieceWiseList[0], perpGeodesics, bestFitInterval[0], bestFitInterval[1]);

                if (bestFitInterval[1] < (perpGeodesics.Count - 1))
                {
                    List<Curve> tempGeodesics = perpGeodesics.GetRange(bestFitInterval[1], (perpGeodesics.Count - bestFitInterval[1]));
                    List<double> tempParameters = perpParameters.GetRange(bestFitInterval[1], (perpGeodesics.Count - bestFitInterval[1]));
                    Vector3d tangent = pieceWiseList[0].TangentAtEnd;
                    double t;
                    tempGeodesics[0].ClosestPoint(pieceWiseList[0].PointAtEnd, out t);
                    tempParameters[0] = t;
                    BestFitPieceWiseGeodesic tempBestFit = new BestFitPieceWiseGeodesic(mesh, tempGeodesics, tempParameters, maxIter / 2, false, 0, threshold, tangent);
                    //var tempOptimizer = new Bobyqa(2, tempBestFit.ComputeError, xl, xu);
                    //var tempResult = tempOptimizer.FindMinimum(startData);

                    pieceWiseList.AddRange(tempBestFit.GenerateSubCurves(startData,xl,xu));

                }

                if (bestFitInterval[0] > 0)
                {

                    List<Curve> tempGeodesics = perpGeodesics.GetRange(0, bestFitInterval[0] + 1);
                    List<double> tempParameters = perpParameters.GetRange(0, bestFitInterval[0] + 1);

                    double t;
                    Vector3d tangent = pieceWiseList[0].TangentAtStart;

                    tempGeodesics[tempGeodesics.Count - 1].ClosestPoint(pieceWiseList[0].PointAtStart, out t);
                    tempParameters[tempGeodesics.Count - 1] = t;
                    BestFitPieceWiseGeodesic tempBestFit = new BestFitPieceWiseGeodesic(mesh, tempGeodesics, tempParameters, maxIter / 2, false, tempGeodesics.Count - 1, threshold, -tangent);
                    //var tempOptimizer = new Bobyqa(2, tempBestFit.ComputeError, xl, xu);
                    //var tempResult = tempOptimizer.FindMinimum(startData);
                    pieceWiseList.AddRange(tempBestFit.GenerateSubCurves(startData,xl,xu));
                }
            }
            return pieceWiseList;
        }

    }

    /// <summary>
    /// Best fit geodesic.
    /// </summary>
    public class BestFitGeodesic {
        // Public properties
        /// <summary>
        /// The selected geodesic.
        /// </summary>
        public Curve selectedGeodesic;

        // Private properties

        protected Mesh mesh;
        protected List<Curve> perpGeodesics;
        protected List<double> perpParameters;
        protected int maxIter;
        protected bool bothDir;
        protected int startIndex;

        /// <summary>
        /// Initializes a new instance of the <see cref="T:MeshGeodesics.BestFitGeodesic"/> class.
        /// </summary>
        /// <param name="aMesh">A mesh.</param>
        /// <param name="pGeods">P geods.</param>
        /// <param name="pParams">P parameters.</param>
        /// <param name="mIter">M iter.</param>
        /// <param name="bDir">If set to <c>true</c> b dir.</param>
        /// <param name="stIndex">St index.</param>
        public BestFitGeodesic(Mesh aMesh, List<Curve> pGeods, List<double> pParams, int mIter, bool bDir, int stIndex)
        {
            mesh = aMesh;
            perpGeodesics = pGeods;
            perpParameters = pParams;
            maxIter = mIter;
            bothDir = bDir;
            startIndex = stIndex;
        }

        /// <summary>
        /// Computes the error.
        /// </summary>
        /// <returns>The error.</returns>
        /// <param name="n">N.</param>
        /// <param name="x">The x coordinate.</param>
        public double ComputeError(int n, double[] x)
        {
            double alpha = x[0];
            Curve curve = perpGeodesics[startIndex];
            Point3d pt = curve.PointAt(perpParameters[startIndex]);
            Vector3d vector = curve.TangentAt(perpParameters[startIndex]);

            MeshPoint mPt = mesh.ClosestMeshPoint(pt, 0.0);
            Vector3d normal = mesh.NormalAt(mPt);
            Vector3d cP = Vector3d.CrossProduct(vector, normal);
            cP.Rotate(alpha, normal);

            Curve newG = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, cP, maxIter).ToNurbsCurve();
            if (bothDir)
            {
                Curve newG2 = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, -cP, maxIter).ToNurbsCurve();
                newG = Curve.JoinCurves(new List<Curve> { newG, newG2 })[0];
            }

            // Assign resulting geodesic to global property for output.
            selectedGeodesic = newG;

            // Calculate error
            double error = 0;

            for (int i = 0; i < perpGeodesics.Count - 1; i++)
            {
                Curve g = perpGeodesics[i];
                CurveIntersections cvInt = Intersection.CurveCurve(newG, g, 0.00001, 0.00001);
                if (cvInt.Count > 0)
                {
                    // Compute distance if intersection is found
                    double param = cvInt[0].ParameterB;
                    Interval domain = new Interval(param, perpParameters[i]);
                    double distance = g.GetLength(domain);
                    // Add squared distance to error
                    error += Math.Pow(distance, 2);
                }
                else
                {
                    // Penalize if no intersection is found on this perp geodesic
                    error += 10;
                }
            }
            return error;
        }
    }


}
