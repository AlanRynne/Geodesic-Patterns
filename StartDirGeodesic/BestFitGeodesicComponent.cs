using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Cureos.Numerics.Optimizers;
            
namespace StartDirGeodesic
{
    public class BestFitGeodesicComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public BestFitGeodesicComponent()
          : base("BestFitGeodesic", "BestGeo",
            "Find the best fitting geodesic on a mesh given a set reference perpendicular geodesics, a set of parameters on such geodesics to measure from.",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh to draw geodesics on", GH_ParamAccess.item);
            pManager.AddCurveParameter("Perp. Geodesics", "Pg", "Set of reference measurement geodesics on a given mesh.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Parameter at Pg", "t", "Parameter of point of reference on Pg", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Maximum Iterations", "Iter", "Integer representing the maximum number of steps for the geodesic curve algorithm", GH_ParamAccess.item, 50);

            pManager[0].Optional = true;
            pManager[1].Optional = true;
            pManager[2].Optional = true;

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Best Fit Geodesic", "BF Geod", "Best fitting geodesic", GH_ParamAccess.list);
            pManager.AddNumberParameter("Best fit result", "R", "Best fit result", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            mesh = null;
            perpGeodesics = new List<Curve>();
            perpParameters = new List<double>();

            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetDataList(1, perpGeodesics)) return;
            if (!DA.GetDataList(2, perpParameters)) return;
            if (!DA.GetData(3, ref maxIter)) return;

            //Only using first variable for now, the extra variable is just to make it work.
            double[] startData = { 0.010, 0 };
            double[] xl = new double[] { -0.050, -1 };
            double[] xu = new double[] { 0.050, 1 };

            var optimizer = new Bobyqa(2, GeodesicFit, xl, xu);
            var result = optimizer.FindMinimum(startData);

            DA.SetDataList(0, new List<Curve> { selectedGeodesic });
            DA.SetDataList(1, result.X);
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
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("f2a4c1b2-d239-4613-adc3-d750c361bc9e"); }
        }


        Mesh mesh;
        List<Curve> perpGeodesics;
        List<double> perpParameters;
        int maxIter = 50;
        Curve selectedGeodesic = null;

        public double fun(int n, double[] x)
        {
            return Math.Pow(x[0], 2) - x[1];
        }

        public double GeodesicFit(int n, double[] x){
            
            double alpha = x[0];
            Curve curve = perpGeodesics[perpGeodesics.Count / 2];
            Point3d pt = curve.PointAt(perpParameters[perpGeodesics.Count / 2]);
            Vector3d vector = curve.TangentAt(perpParameters[perpGeodesics.Count / 2]);

            MeshPoint mPt = mesh.ClosestMeshPoint(pt, 0.0);
            Vector3d normal = mesh.NormalAt(mPt);
            Vector3d cP = Vector3d.CrossProduct(vector, normal);
            cP.Rotate(alpha, normal);

            Curve newG = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, cP, maxIter).ToNurbsCurve();
            Curve newG2 = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, pt, -cP, maxIter).ToNurbsCurve();
            newG = Curve.JoinCurves(new List<Curve> { newG, newG2 })[0];

            selectedGeodesic = newG;

            double error = 0;

            for (int i = 0; i < perpGeodesics.Count-1; i++)
            {
                Curve g = perpGeodesics[i];
                CurveIntersections cvInt = Intersection.CurveCurve(newG, g,0.00001,0.00001);
                if (cvInt.Count > 0)
                {
                    double param = cvInt[0].ParameterB;
                    Interval domain = new Interval(param, perpParameters[i]);
                    double distance = g.GetLength(domain);

                    error += Math.Pow(distance, 2);
                }
                else
                {
                    // Penalize if no intersection is found on this perp geodesic
                    error += 1000000000;
                }
            }

            return error;
        }
    }
}