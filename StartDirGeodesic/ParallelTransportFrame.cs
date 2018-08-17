using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MeshGeodesics
{
    public class ParallelTransportFrameComponent : GH_Component
    {
        public ParallelTransportFrameComponent()
          : base("Parallel Transport Frame", "TNB Frame",
                 "Move a vector V(i) along a curve on a mesh such as it remains tangent to the surface S and parallel to V(i-1) ",
            "Alan", "Geodesic Patterns")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh the curve should lie on", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector", "V", "The vector to parallel transport along the curve", GH_ParamAccess.item);
            pManager.AddPointParameter("Point list", "P", "Sample points on the curve", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {

            pManager.AddVectorParameter("Transported Vectors", "Vt", "List representing the transported vectors, INCLUDING the initial vector", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            Vector3d startDirection = Vector3d.Unset;
            List<Point3d> points = new List<Point3d>();

            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetData(1, ref startDirection)) return;
            if (!DA.GetDataList(2, points)) return;

            List<Vector3d> vectors = MeshGeodesicMethods.VectorParallelTransport(startDirection, points, mesh);

            DA.SetDataList(0, vectors);
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Properties.Resources.ParTrans;
            }
        }

        public override Guid ComponentGuid
        {
            get { return new Guid("41f665d9-5394-4a9f-9afd-44b17cb2ad52"); }
        }
    }
}