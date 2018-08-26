using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Geometry.Collections;

using Cureos.Numerics.Optimizers;

#pragma warning disable 1591

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
    /// Best fit geodesic.
    /// </summary>
    public class BestFitGeodesic
    {
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


    /// <summary>
    /// Best fit piece-wise geodesic.
    /// </summary>
    public class BestFitPieceWiseGeodesic : BestFitGeodesic
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
            bestFitInterval = new int[] { };
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
            if (angle >= 0.1 * Math.PI) cP = -cP;

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
            for (int k = (distances.Count - 1); k >= 2; k--)
            {
                for (int i = 0; i < (distances.Count - k); i++)
                {
                    //Check if interval i->k is within bounds
                    bool flag = true;
                    for (int j = i; j < i + k; j++)
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
                        bestFitInterval = new int[] { i, i + k };
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
        /// <param name="type">Generation type: 0 for doubleside, 1 for end side, 2 for start side of curve</param>
        public List<Curve> GenerateSubCurves(double[] _startData, double[] _xl, double[] _xu, int type)
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
                switch (type)
                {
                    // End point
                    case 1:
                        pieceWiseList[0] = CutCurveBetweenPerpIndexes(pieceWiseList[0], perpGeodesics, 0, bestFitInterval[1]);
                        break;
                    // Start point
                    case -1:
                        pieceWiseList[0] = CutCurveBetweenPerpIndexes(pieceWiseList[0], perpGeodesics, bestFitInterval[0], perpGeodesics.Count - 1);
                        break;
                    default:
                        pieceWiseList[0] = CutCurveBetweenPerpIndexes(pieceWiseList[0], perpGeodesics, bestFitInterval[0], bestFitInterval[1]);
                        break;
                }

                if (bestFitInterval[1] < (perpGeodesics.Count - 1))
                {
                    List<Curve> tempGeodesics = perpGeodesics.GetRange(bestFitInterval[1], (perpGeodesics.Count - bestFitInterval[1]));
                    List<double> tempParameters = perpParameters.GetRange(bestFitInterval[1], (perpGeodesics.Count - bestFitInterval[1]));
                    Vector3d tangent = pieceWiseList[0].TangentAtEnd;
                    double t;
                    tempGeodesics[0].ClosestPoint(pieceWiseList[0].PointAtEnd, out t);
                    tempParameters[0] = t;
                    BestFitPieceWiseGeodesic tempBestFit = new BestFitPieceWiseGeodesic(mesh, tempGeodesics, tempParameters, Convert.ToInt32(maxIter * 0.8), false, 0, threshold, tangent);
                    var tempOptimizer = new Bobyqa(2, tempBestFit.ComputeError, xl, xu);
                    var tempResult = tempOptimizer.FindMinimum(startData);

                    pieceWiseList.AddRange(tempBestFit.GenerateSubCurves(startData, xl, xu, 1));

                }

                if (bestFitInterval[0] > 0)
                {

                    List<Curve> tempGeodesics = perpGeodesics.GetRange(0, bestFitInterval[0] + 1);
                    List<double> tempParameters = perpParameters.GetRange(0, bestFitInterval[0] + 1);

                    double t;
                    Vector3d tangent = pieceWiseList[0].TangentAtStart;
                    tangent.Rotate(Math.PI, mesh.NormalAt(mesh.ClosestMeshPoint(pieceWiseList[0].PointAtStart, 0.0)));

                    tempGeodesics[tempGeodesics.Count - 1].ClosestPoint(pieceWiseList[0].PointAtStart, out t);
                    tempParameters[tempGeodesics.Count - 1] = t;
                    BestFitPieceWiseGeodesic tempBestFit = new BestFitPieceWiseGeodesic(mesh, tempGeodesics, tempParameters, Convert.ToInt32(maxIter * 0.8), false, tempGeodesics.Count - 1, threshold, tangent);
                    var tempOptimizer = new Bobyqa(2, tempBestFit.ComputeError, xl, xu);
                    var tempResult = tempOptimizer.FindMinimum(startData);
                    //pieceWiseList.AddRange(tempBestFit.GenerateSubCurves(startData,xl,xu,-1));
                    pieceWiseList.Add(tempBestFit.selectedGeodesic);
                }
            }
            return pieceWiseList;
        }

    }

    /// <summary>
    /// Geodesic 1-pattern from level sets on triangular meshes.
    /// </summary>
    public class GeodesicsFromLevelSets
    {
        // Public properties
        public Mesh Mesh { get => _mesh; set => _mesh = value; }
        public List<double> VertexValues { get => _vertexValues; set => _vertexValues = value; }
        public double DesiredWidth { get => _desiredWidth; set => _desiredWidth = value; }
        public double Lambda { get => _lambda; set => _lambda = value; }
        public double Nu { get => _nu; set => _nu = value; }
        public List<double> VertexVoronoiArea { get => _vertexVoronoiArea; set => _vertexVoronoiArea = value; }
        public List<Vector3d> Gradient { get => _gradient; set => _gradient = value; }

        // Private fields
        Mesh _mesh;
        List<double> _vertexVoronoiArea;
        List<double> _vertexValues;
        List<Vector3d> _gradient;
        double _desiredWidth;
        double _lambda;
        double _nu;

        // Constructor
        public GeodesicsFromLevelSets(Mesh mesh, List<double> vertexValues, double desiredWidth, double lambda, double nu)
        {
            _mesh = mesh;
            _vertexValues = vertexValues;
            _desiredWidth = desiredWidth;
            _nu = nu;
            _lambda = lambda;
            VertexVoronoiArea = ComputeVertexVoronoiArea();
        }


        /// <summary>
        /// Computes the vertex voronoi area of every vertex in the mesh.
        /// </summary>
        /// <returns>The vertex voronoi area.</returns>
        public List<double> ComputeVertexVoronoiArea()
        {
            List<double> areas = new List<double>();
            for (int i = 0; i < _mesh.TopologyVertices.Count; i++)
            {
                // Compute the voronoi area of a vertex
                double VoronoiArea = 0.0;
                int topologyVertexIndex = _mesh.TopologyVertices.TopologyVertexIndex(i);
                Debug.Print("Connected Faces {0}", _mesh.TopologyVertices.ConnectedFaces(topologyVertexIndex).Length);
                foreach (int index in _mesh.TopologyVertices.ConnectedFaces(topologyVertexIndex))
                {
                    MeshFace face = _mesh.Faces[index];
                    Point3d Vi = _mesh.Vertices[face.A];
                    Point3d Vj = _mesh.Vertices[face.B];
                    Point3d Vk = _mesh.Vertices[face.C];

                    double faceArea = 0.5 * Vector3d.CrossProduct(Vj - Vi, Vk - Vi).Length;
                    // For now, the voronoi area is considered to be a third of the total area of triangles surrounding the vertex.
                    VoronoiArea += faceArea / 3;
                }
                areas.Add(VoronoiArea);
            }
            return areas;
        }

        /// <summary>
        /// Computes the PER FACE gradient of a piecewise scalar function on the vertex of a mesh
        /// </summary>
        /// <returns>The gradient of the function as a constant vector per face of the mesh</returns>
        public void ComputeGradient()
        {
            List<Vector3d> gradientVectors = new List<Vector3d>();

            // Gradient is calculated PER FACE
            foreach (MeshFace face in _mesh.Faces)
            {
                // Vertices
                Point3d i = _mesh.Vertices[face.A];
                Point3d j = _mesh.Vertices[face.B];
                Point3d k = _mesh.Vertices[face.C];
                // Edges
                Vector3d eij = j - i;
                Vector3d ejk = k - j;
                Vector3d eki = i - k;
                // Vertex values
                double gi = _vertexValues[face.A];
                double gj = _vertexValues[face.B];
                double gk = _vertexValues[face.C];
                // Face area & normal
                double faceArea = Vector3d.CrossProduct(eij, k - i).Length / 2;
                Vector3d faceNormal = Vector3d.CrossProduct(ejk, eki) / (2 * faceArea);
                // Compute 90 degree rotated grad
                Vector3d rotatedGrad = (gi * ejk + gj * eki + gk * eij) / (2 * faceArea);
                // Rotate -90 degrees to obtain face grad
                Vector3d grad = Vector3d.CrossProduct(rotatedGrad, faceNormal);
                // Add gradient vector to list
                gradientVectors.Add(grad);
            }
            _gradient = gradientVectors;
        }

        /// <summary>
        /// Compute the PER VERTEX divergence of a face-based gradient vector field.
        /// </summary>
        /// <returns>The PER VERTEX divergence</returns>
        /// <param name="normalize">If set to <c>true</c>, normalize the vector field.</param>
        public List<double> ComputeDivergence(bool normalize)
        {
            // Divergence of a face based vector field computed PER VERTEX
            //throw new NotImplementedException("ComputeDivergence() has not been implemented yet");
            List<double> divergence = new List<double>();
            for (int i = 0; i < _mesh.Vertices.Count; i++)
            {
                Point3d vertex = _mesh.Vertices[i];
                double divV = 0.0;
                foreach (int faceIndex in _mesh.TopologyVertices.ConnectedFaces(_mesh.TopologyVertices.TopologyVertexIndex(i)))
                {
                    Vector3d grad = Gradient[faceIndex];
                    if (normalize) grad.Unitize();
                    Vector3d faceNormal = _mesh.FaceNormals[faceIndex];

                    Vector3d oppositeEdge = Vector3d.Unset;
                    // Find the opposite edge of the current vertex in this face
                    if (_mesh.Vertices[_mesh.Faces[faceIndex].A] == vertex) oppositeEdge = _mesh.Vertices[_mesh.Faces[faceIndex].C] - _mesh.Vertices[_mesh.Faces[faceIndex].B];
                    if (_mesh.Vertices[_mesh.Faces[faceIndex].B] == vertex) oppositeEdge = _mesh.Vertices[_mesh.Faces[faceIndex].A] - _mesh.Vertices[_mesh.Faces[faceIndex].C];
                    if (_mesh.Vertices[_mesh.Faces[faceIndex].C] == vertex) oppositeEdge = _mesh.Vertices[_mesh.Faces[faceIndex].B] - _mesh.Vertices[_mesh.Faces[faceIndex].A];

                    double faceDiv = Vector3d.Multiply(grad, Vector3d.CrossProduct(faceNormal, oppositeEdge));
                    divV += faceDiv;    
                }
                divergence.Add(-divV);
            }
            return divergence;
        }

        /// <summary>
        /// Geodesic curvature fit function
        /// </summary>
        /// <returns>Fk - Global geodesic fittness of the scalar field</returns>
        public double FitK()
        {
            // FitK = Sumation of all vFk's
            double Fk = 0.0;
            // Compute NORMALIZED divergence (the divergence of the normalized gradient)
            List<double> divergence = ComputeDivergence(true);
            for (int i = 0; i < _mesh.Vertices.Count; i++)
            {
                // FK in vertex V -> vFk = (Voronoi Area of Vertex) * (Divergence of Normalized Field)^2
                Fk += VertexVoronoiArea[i] * Math.Pow(divergence[i], 2);
            }
            return Fk;
        }

        /// <summary>
        /// Regularization fitness function
        /// </summary>
        /// <returns>F△ - The regularization fittnes value of the scalar field</returns>
        public double FitA()
        {
            // F△ = (Area of Mesh) * (Sumation of all F△'s)
            double Fa = 0.0; // F△
            double meshArea = AreaMassProperties.Compute(_mesh).Area;
            // Compute divergence
            List<double> divergence = ComputeDivergence(false);

            for (int i = 0; i < _mesh.Vertices.Count; i++)
            {
                // F△(v) = (Voronoi Area of Vertex) * (Divergence of Field)^2
                Fa += VertexVoronoiArea[i] * Math.Pow(divergence[i], 2);
            }

            //Multiply FitA result by Mesh Area
            Fa *= meshArea;

            // Output result
            return Fa;
                
        }

        /// <summary>
        /// Width fitness function
        /// </summary>
        /// <returns>Fw - Global constant width fitness value of the scalar field.</returns>
        /// <param name="w">The desired width.</param>
        /// <param name="h">Numerical value of separation in scalar field (usually 1)</param>
        public double FitW(double w, double h)
        {
            // Compute Equal Width fitness of Gradient Field
            double Fw = 0.0;
            for (int i = 0; i < _mesh.Faces.Count; i++)
            {
                MeshFace face = _mesh.Faces[i];
                Vector3d faceVector = Gradient[i];
                // Vertices
                Point3d Vi = _mesh.Vertices[face.A];
                Point3d Vj = _mesh.Vertices[face.B];
                Point3d Vk = _mesh.Vertices[face.C];

                double faceArea = Vector3d.CrossProduct(Vj - Vi, Vk - Vi).Length / 2 ;

                double vFw = faceArea * Math.Pow(faceVector.Length - (h / w), 2);

                Fw += vFw;
            } 
            return Fw;
        }

        int computeCount = 0;
        /// <summary>
        /// Optimization Function
        /// </summary>
        /// <returns>Value to minimize</returns>
        /// <param name="i">Number of variables in <paramref name="x"/></param>
        /// <param name="x">Array containing the variable values</param>
        public double Compute(int i, double[] x)
        {
            //throw new NotImplementedException("Draw Level Set Curves is not implemented yet");
            _vertexValues = new List<double>(x);
            ComputeGradient();
            double fk = FitK();
            double fa = FitA();
            double fw = FitW(_desiredWidth,1);
            double Fmin = fk + _lambda * fa + _nu * fw;
            if (computeCount%100 == 0) Debug.Print("Iter {0} Fmin = {1}",computeCount, Fmin);
            computeCount++;
            return Fmin;
        }

        /// <summary>
        /// Draws the level set curves.
        /// </summary>
        public DataTree<Line> DrawLevelSetCurves(List<double> plotValues)
        {
            DataTree<Line> plotLineTree = new DataTree<Line>();
            // Iterate each face once
            foreach (MeshFace face in _mesh.Faces)
            {
                // Iterate each value once per face 
                for (int i = 0; i < plotValues.Count; i++)
                {
                    Line levelLine;
                    double value = plotValues[i];

                    bool success = CheckLevelSetInFace(value, face, out levelLine);

                    if (success) plotLineTree.Add(levelLine, new Grasshopper.Kernel.Data.GH_Path(i));
                }
            }
            return plotLineTree;

        }

        bool CheckLevelSetInFace(double level, MeshFace face, out Line line)
        {
            List<double> vertexValues = new List<double> { VertexValues[face.A], VertexValues[face.B], VertexValues[face.C] };
            List<Point3d> faceVertices = new List<Point3d> { _mesh.Vertices[face.A], _mesh.Vertices[face.B], _mesh.Vertices[face.C] };

            List<int> above = new List<int>();
            List<int> below = new List<int>();

            for (int i = 0; i < vertexValues.Count; i++)
            {
                if (vertexValues[i] < level) below.Add(i);
                else above.Add(i);
            }

            if (above.Count == 3 || below.Count == 3)
            {
                // Triangle is above or below level
                line = new Line();
                return false;
            }
            else
            {
                // Triangle intersects level
                List<Point3d> intersectionPoints = new List<Point3d>();

                foreach (int i in above)
                {
                    foreach (int j in below)
                    {
                        double diff = vertexValues[i] - vertexValues[j];
                        double desiredDiff = level - vertexValues[j];
                        double unitizedDistance = desiredDiff / diff;
                        Vector3d edgeV = faceVertices[i] - faceVertices[j];
                        Point3d levelPoint = faceVertices[j] + edgeV * unitizedDistance;
                        intersectionPoints.Add(levelPoint);
                    }
                }
                line = new Line(intersectionPoints[0], intersectionPoints[1]);
                return true;
            }
        }

    }
}
