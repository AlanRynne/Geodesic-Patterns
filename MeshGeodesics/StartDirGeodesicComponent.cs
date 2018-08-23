using System;
using System.Collections.Generic;
using System.Diagnostics;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;


namespace MeshGeodesics
{
    /// <summary>
    /// Start dir geodesic component.
    /// </summary>
    public class StartDirGeodesicComponent : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="T:MeshGeodesics.StartDirGeodesicComponent"/> class.
        /// </summary>
        public StartDirGeodesicComponent()
          : base("Start-Dir Geodesic", "Geod-SD",
            "StartDirGeodesic description",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers the input parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Max Iterations", "Max. Iter", "Set maximum iterations", GH_ParamAccess.item,5);
            pManager.AddMeshParameter("Mesh", "M", "Mesh to draw geodesics on", GH_ParamAccess.item);
            pManager.AddPointParameter("Start Point", "P", "Starting points of geodesics", GH_ParamAccess.list);
            pManager.AddVectorParameter("Start Direction", "V", "Initial direction of geodesics", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Both directions", "Both", "Create geodesics both positive and negative directions of V", GH_ParamAccess.item, false);
        }

        /// <summary>
        /// Registers the output parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic curves", "G", "List of calculated geodesic curves", GH_ParamAccess.list);
            pManager.AddVectorParameter("Geodesic vectors", "Vg", "List of all calculated vectors for geodesic field", GH_ParamAccess.tree);

        }

        /// <summary>
        /// Solves the instance.
        /// </summary>
        /// <param name="DA">Da.</param>
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
        /// Gets the icon.
        /// </summary>
        /// <value>The icon.</value>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.StartDir;
            }
        }

        /// <summary>
        /// Gets the component GUID.
        /// </summary>
        /// <value>The component GUID.</value>
        public override Guid ComponentGuid
        {
            get { return new Guid("6b866c6f-a1ee-41b9-afed-f366a1bbe8f8"); }
        }
    }

}
