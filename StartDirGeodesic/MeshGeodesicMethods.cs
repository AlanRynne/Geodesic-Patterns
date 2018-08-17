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
    public static class MeshGeodesicMethods
    {
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

        public static List<Curve> ComputeGeodesicPatternByParralelTransport(Mesh mesh, Curve startGeodesic, int count, double alpha, int iter)
        {
            //Sample curves using provided 'count'
            double[] parameters = startGeodesic.DivideByCount(count,true);

            //At each sample point obtain the mesh normal and the curve tangent
            List<Point3d> samplePoints = new List<Point3d>();
            List<Vector3d> tangentVectors = new List<Vector3d>();
            List<Vector3d> normalVectors = new List<Vector3d>();
            foreach (double t in parameters)
            {
                Point3d pt = startGeodesic.PointAt(t);
                Vector3d tang = startGeodesic.TangentAt(t);
                Vector3d normal = mesh.NormalAt(mesh.ClosestMeshPoint(pt,0.0));
                samplePoints.Add(pt);
                tangentVectors.Add(tang);
                normalVectors.Add(normal);
            }

            //At the starting point of the curve: obtain the cross product of tangent and normal.
            Vector3d CP = Vector3d.CrossProduct(tangentVectors[0], normalVectors[0]);

            //Rotate the vector around the normal using the specified 'alpha' angle in radians
            CP.Rotate(alpha, normalVectors[0]);

            //Parallel transport the vector along the rest of the sample parameters.
            List<Vector3d> transportedVectors = VectorParallelTransport(CP, samplePoints,mesh);
                
            //Generate geodesics using the sample points and the transported vectors.
            List<Curve> geodesicPattern = new List<Curve>();
            for (int i = 0; i < samplePoints.Count;i++)
            {
                Curve geod = getGeodesicCurveOnMesh(mesh, samplePoints[i], transportedVectors[i], iter).ToNurbsCurve();
                Curve geodInverse = getGeodesicCurveOnMesh(mesh, samplePoints[i], -transportedVectors[i], iter).ToNurbsCurve();
                Curve fullGeodesic = Curve.JoinCurves(new List<Curve>{geod, geodInverse})[0];
                geodesicPattern.Add(fullGeodesic);
            }

            //Return geodesic patterns 
            return geodesicPattern;
        }

        public struct ParallelTransportFrame
        {
            public Vector3d T;
            public Vector3d N;
            public Vector3d B;
            public Point3d position;
        }

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

        public static List<Vector3d> VectorParallelTransport(Vector3d vector, List<Point3d> points, Mesh mesh)
        {
            List<Vector3d> transportedVectors = new List<Vector3d>();
            transportedVectors.Add(vector);

            for (int i = 1; i < points.Count;i++)
            {
                Vector3d CP = Vector3d.CrossProduct(transportedVectors[i - 1], mesh.NormalAt(mesh.ClosestMeshPoint(points[i - 1],0.0)));
                Vector3d newV = Vector3d.CrossProduct(mesh.NormalAt(mesh.ClosestMeshPoint(points[i - 1],0.0)), CP);
                transportedVectors.Add(newV);
            }

            return transportedVectors;
        }
    }

    public class BestFitGeodesic {
        // Public properties
        public Curve selectedGeodesic;

        // Private properties
        Mesh mesh;
        List<Curve> perpGeodesics;
        List<double> perpParameters;
        int maxIter;
        bool bothDir;
        int startIndex;

        public BestFitGeodesic(Mesh aMesh, List<Curve> pGeods, List<double> pParams, int mIter, bool bDir, int stIndex)
        {
            mesh = aMesh;
            perpGeodesics = pGeods;
            perpParameters = pParams;
            maxIter = mIter;
            bothDir = bDir;
            startIndex = stIndex;
        }
        // Find the best fitting geodesic curve for a set of 
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
