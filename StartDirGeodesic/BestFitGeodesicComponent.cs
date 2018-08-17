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
            pManager.AddBooleanParameter("Both Directions", "BothDir", "Generate the geodesic on both directions",GH_ParamAccess.item,false);
            pManager.AddIntegerParameter("Start Point Index", "StIndex", "Index of starting point to use from the t' list", GH_ParamAccess.item);
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

            // Set the input data
            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetDataList(1, perpGeodesics)) return;
            if (!DA.GetDataList(2, perpParameters)) return;
            if (!DA.GetData(3, ref maxIter)) return;
            if (!DA.GetData(4, ref bothDir)) return;
            if (!DA.GetData(5, ref startIndex)) return;

            // Data validation
            if (maxIter == 0) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "MaxIter cannot be 0");
            if (!mesh.IsValid) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Mesh is invalid");
            if (perpGeodesics.Count < 2) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "There must be at least 2 perpendicular geodesics");
            if (perpParameters.Count != perpGeodesics.Count) AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Curves and Params list must have the same number of items");

            // Generate Initial values and variable Bounds for the optimization problem.
            // Only using first variable for now, the extra variable is just to make it work.
            double[] startData = { 0.010, 0 };
            double[] xl = new double[] { -0.080, -1 };
            double[] xu = new double[] { 0.080, 1 };

            BestFitGeodesic bestFitG = new BestFitGeodesic(mesh, perpGeodesics, perpParameters, maxIter, bothDir, startIndex);
            var optimizer = new Bobyqa(2, bestFitG.Compute, xl, xu);
            var result = optimizer.FindMinimum(startData);

            // Output data
            DA.SetDataList(0, new List<Curve> { bestFitG.selectedGeodesic });
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
                return Properties.Resources.BestFitGeo;
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
    }
}