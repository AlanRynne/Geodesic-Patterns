using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MeshGeodesics
{
    /// <summary>
    /// Geodesic pattern by Parallel Transport.
    /// </summary>
    public class GeodesicPatternByPT : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="T:MeshGeodesics.GeodesicPatternByPT"/> class.
        /// </summary>
        public GeodesicPatternByPT()
          : base("Geodesic Pattern by PT", "G x PT",
            "Generate a 1-pattern of geodesic curves by Parallel Transport Method",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers the input parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh to use for geodesic generation", GH_ParamAccess.item);
            pManager.AddCurveParameter("Start Geodesic", "StGeod", "Starting geodesic curve on the mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "C", "Amount of curves to be generated", GH_ParamAccess.item);
            pManager.AddNumberParameter("Angle", "A", "Rotation Angle in Radians", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Max. Iterations", "MaxIter", "Maximum Iterations to compute each geodesic",GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers the output parameters.
        /// </summary>
        /// <param name="pManager">P manager.</param>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Pattern", "GP", "Generated geodesic pattern",GH_ParamAccess.list);

        }

        /// <summary>
        /// Solves the instance.
        /// </summary>
        /// <param name="DA">Da.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            Curve startGeod = null;
            int Count = 0;
            double Angle = 0.0;
            int Iter = 0;

            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetData(1, ref startGeod)) return;
            if (!DA.GetData(2, ref Count)) return;
            if (!DA.GetData(3, ref Angle)) return;
            if (!DA.GetData(4, ref Iter)) return;


            List<Curve> geodesicPattern = MeshGeodesicMethods.ComputeGeodesicPatternByParralelTransport(mesh, startGeod, Count, Angle, Iter);

            DA.SetDataList(0, geodesicPattern);
        }

        /// <summary>
        /// Gets the icon.
        /// </summary>
        /// <value>The icon.</value>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Properties.Resources.GeodxPT;
            }
        }

        /// <summary>
        /// Gets the component GUID.
        /// </summary>
        /// <value>The component GUID.</value>
        public override Guid ComponentGuid
        {
            get { return new Guid("98d6c735-a2eb-4f06-aba1-bce032b9aad7"); }
        }
    }
}
