using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace StartDirGeodesic
{
    public class GeodesicPatternByPT : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public GeodesicPatternByPT()
          : base("Geodesic Pattern by PT", "G x PT",
            "Generate a 1-pattern of geodesic curves by Parallel Transport Method",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh to use for geodesic generation", GH_ParamAccess.item);
            pManager.AddCurveParameter("Start Geodesic", "StGeod", "Starting geodesic curve on the mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Count", "C", "Amount of curves to be generated", GH_ParamAccess.item);
            pManager.AddNumberParameter("Angle", "A", "Rotation Angle in Radians", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Max. Iterations", "MaxIter", "Maximum Iterations to compute each geodesic",GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Pattern", "GP", "Generated geodesic pattern",GH_ParamAccess.list);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
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
            get { return new Guid("98d6c735-a2eb-4f06-aba1-bce032b9aad7"); }
        }
    }
}
