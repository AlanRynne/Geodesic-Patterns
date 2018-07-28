using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace StartDirGeodesic
{
    public class ParallelTransportFrameComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public ParallelTransportFrameComponent()
          : base("Parallel Transport Frame", "TNB Frame",
                 "Move a vector V(i) along a curve on a mesh such as it remains tangent to the surface S and parallel to V(i-1) ",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh the curve should lie on", GH_ParamAccess.item);
            pManager.AddVectorParameter("Vector", "V", "The vector to parallel transport along the curve", GH_ParamAccess.item);
            pManager.AddPointParameter("Point list", "P", "Sample points on the curve", GH_ParamAccess.list);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Tangent", "T", "Tangent vector at each point", GH_ParamAccess.list);
            pManager.AddVectorParameter("Normal", "N", "Normal vector at each point", GH_ParamAccess.list);
            pManager.AddVectorParameter("Binormal", "B", "Binormal vector at each point", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            Vector3d startDirection = Vector3d.Unset;
            List<Point3d> points = new List<Point3d>();

            List<ParallelTransportFrame> frames = new List<ParallelTransportFrame>();

            if (!DA.GetData(0, ref mesh)) return;
            if (!DA.GetData(1, ref startDirection)) return;
            if (!DA.GetDataList(2, points)) return;

            frames = ParallelTransportFrames(startDirection, points);

            List<Vector3d> T = new List<Vector3d>();
            List<Vector3d> N = new List<Vector3d>();
            List<Vector3d> B = new List<Vector3d>();

            foreach (ParallelTransportFrame frame in frames)
            {
                T.Add(frame.T);
                N.Add(frame.N);
                B.Add(frame.B);
            }

            DA.SetDataList(0, T);
            DA.SetDataList(1, N);
            DA.SetDataList(2, B);
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
            get { return new Guid("41f665d9-5394-4a9f-9afd-44b17cb2ad52"); }
        }

        // Original Parallel Transport Frames algorithm

        //void ParallelTransportFrames::create(Vec3 V) {
        //    Vec3 V_prev = V;
        //    for (int i = 0; i < points.size() - 2; ++i)
        //    {
        //        Vec3 t0 = (points[i + 1] - points[i]).getNormalized();
        //        Vec3 t1 = (points[i + 2] - points[i + 1]).getNormalized();

        //        Vec3 B = cross(t0, t1);
        //        if (B.length() <= 0.01)
        //        {
        //            V = V_prev;
        //        }
        //        else
        //        {
        //            B.normalize();
        //            float theta = acosf(dot(t0, t1));
        //            V.rotate(theta, B);
        //        }

        //        Vec3 binorm = cross(t0, V).getNormalized();
        //        ParallelTransportFrame frame = { t0, V, binorm, points[i] };
        //        frames.push_back(frame);
        //        V_prev = V;
        //    }
        //}

        struct ParallelTransportFrame
        {
            public Vector3d T;
            public Vector3d N;
            public Vector3d B;
            public Point3d position;
        }

        List<ParallelTransportFrame> ParallelTransportFrames(Vector3d V, List<Point3d> pts)
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
    }
}