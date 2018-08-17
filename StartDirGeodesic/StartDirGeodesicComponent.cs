using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;


namespace StartDirGeodesic
{
    public class StartDirGeodesicComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public StartDirGeodesicComponent()
          : base("Start-Dir Geodesic", "Geod-SD",
            "StartDirGeodesic description",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Max Iterations", "Max. Iter", "Set maximum iterations", GH_ParamAccess.item,5);
            pManager.AddMeshParameter("Mesh", "M", "Mesh to draw geodesics on", GH_ParamAccess.item);
            pManager.AddPointParameter("Start Point", "P", "Starting points of geodesics", GH_ParamAccess.list);
            pManager.AddVectorParameter("Start Direction", "V", "Initial direction of geodesics", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Both directions", "Both", "Create geodesics both positive and negative directions of V", GH_ParamAccess.item, false);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic curves", "G", "List of calculated geodesic curves", GH_ParamAccess.list);
            pManager.AddVectorParameter("Geodesic vectors", "Vg", "List of all calculated vectors for geodesic field", GH_ParamAccess.tree);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // Input variables
            int iter = 0;
            Mesh mesh = new Mesh();
            List<Point3d> startPoints = new List<Point3d>();
            List<Vector3d> startDirections = new List<Vector3d>();
            // Output holders
            List<Curve> polyList = new List<Curve>();
            bool bothDir = false;

            // Value setting
            if (!DA.GetData(0, ref iter)) return;
            if (!DA.GetData(1, ref mesh)) return;
            if (!DA.GetDataList(2, startPoints)) return;
            if (!DA.GetDataList(3, startDirections)) return;
            if (!DA.GetData(4, ref bothDir)) return;

            // Value checks
            if (startPoints.Count != startDirections.Count) {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Start point and vector list must be the same length");
            }

            for (int i = 0; i < startPoints.Count; i++){
                Curve crv;
                Polyline pl = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, startPoints[i], startDirections[i], iter);

                if (bothDir) {
                    Polyline pl2 = MeshGeodesicMethods.getGeodesicCurveOnMesh(mesh, startPoints[i], -startDirections[i], iter);
                    crv = Curve.JoinCurves(new List<Curve>{pl.ToNurbsCurve(),pl2.ToNurbsCurve()})[0];

                } else {
                    crv = pl.ToNurbsCurve();
                }
                polyList.Add(crv);
            }


            // Output results
            DA.SetDataList(0, polyList);
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
                return Properties.Resources.StartDir;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("6b866c6f-a1ee-41b9-afed-f366a1bbe8f8"); }
        }
    }

}
