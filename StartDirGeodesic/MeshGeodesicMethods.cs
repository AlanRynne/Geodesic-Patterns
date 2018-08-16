using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace StartDirGeodesic
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
        public double Compute(int n, double[] x)
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
            else
            {
                // keep newG intact
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
