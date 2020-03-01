using System;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace MeshGeodesics
{
    public class PanelStressComponent : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public PanelStressComponent()
          : base("Panel Stress", "Stress",
            "Panel Stress calculations",
            "Alan", "Geodesic Patterns")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve Pattern", "G", "Geodesic Pattern representing the central line of each panel.", GH_ParamAccess.list);
            pManager.AddSurfaceParameter("Panels", "P", "Panels previously generated from G", GH_ParamAccess.list);
            pManager.AddNumberParameter("Thickness", "h", "Specified panel thickness", GH_ParamAccess.item);
            pManager.AddNumberParameter("Young's m.", "E", "Material young's modulus", GH_ParamAccess.item);
            pManager.AddNumberParameter("Shear m.", "G", "Material shear modulus", GH_ParamAccess.item);

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddBooleanParameter("Admissible", "Ok", "Returns true if panel pattern is admissible, false if not", GH_ParamAccess.item);
            pManager.AddNumberParameter("C","C", "C constant", GH_ParamAccess.list);
            pManager.AddNumberParameter("Panel Tensile Stress", "Ts", "Panel Tensile Stress", GH_ParamAccess.list);
            pManager.AddNumberParameter("Panel Bending Stress", "Bs", "Panel Bending Stress", GH_ParamAccess.list);
            pManager.AddNumberParameter("Panel Shear Stress", "Ss", "Panel Shear Stress", GH_ParamAccess.list);


        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<Curve> pattern = new List<Curve>();
            List<Surface> panels = new List<Surface>();
            double h = 0;
            double E = 0;
            double G = 0;

            if (!DA.GetDataList(0, pattern)) return;
            if (!DA.GetDataList(1, panels)) return;
            if (!DA.GetData(2, ref h)) return;
            if (!DA.GetData(3, ref E)) return;
            if (!DA.GetData(4, ref G)) return;


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
            get { return new Guid("2bb54096-d2d9-41ff-b233-db993d8e58e8"); }
        }

        public double ComputeCurveTorsion(Curve c, int sampleCount)
        {
            double totalTorsion = 0;

            // Make sure interval is 0 to 1
            c.Domain = new Interval(0.0, 1.0);
            // Compute step size
            double step = 1 / sampleCount;
            // Iterate for all values of from 0 to 1
            for (double i = 0; i < 1; i += step)
            {
                Plane p1, p2;
                c.PerpendicularFrameAt(i, out p1);
                c.PerpendicularFrameAt(i + step, out p2);

                Vector3d cP = Vector3d.CrossProduct(p1.XAxis, p1.ZAxis);
                Vector3d tV = Vector3d.CrossProduct(p2.ZAxis, cP);
                double angle = Vector3d.VectorAngle(p2.XAxis, tV);
                totalTorsion += angle;
            }
            // Return unit length torsion
            return totalTorsion / c.GetLength();
        }
        public struct PanelResults
        {
            public bool Admissible;
            public double CurrentWidth;
            public double MaxStrain;
            public double MaxWidth;
            public double MaxBending;
            public double MaxShear;

        }
        // Strain in panels
        public PanelResults CheckPanelAdmisibility(Surface p, Curve c, double h, double E, double G)
        {
            double K = findMaxGaussianCurvatureInPanel(p, 0.05); // Gaussian Curvature (compute)
            double d = 0; // Width (calculate)
            E = 0; // Young Modulus

            PanelResults results = new PanelResults();

            double rho = 1/(Math.Sqrt(Math.Abs(K)));
            double epsilon = 0.5 * Math.Pow((d / (2 * rho)), 2);

            double sigma = E * rho; // Tensile stress

            // Find maximum tensile stress and assign to sigmaMax
            // Find minimum radius of curvature (rho).
            double sigmaMax = 0;
            double rhoMin = 0;

            // Use maximum tensile stress to obtain C constant
            double C = Math.Sqrt(2 * sigmaMax / E);
            // Compute dMax
            double dMax = 2 * rhoMin * C;

            // Check width admissibility
            if (d < dMax)
            {
                results.Admissible = true;
            } else {
                results.Admissible = false;
            }
            // Assign values to results
            results.MaxStrain = sigmaMax;
            results.MaxWidth = dMax;
            results.CurrentWidth = d;

            // Compute remaining stresses & assign to results
            int steps = 30;
            results.MaxBending = ComputePanelBendingStress(c, h, E, steps);
            results.MaxShear = ComputePanelShearStress(h, G, ComputeCurveTorsion(c, steps));

            // Return the results
            return results;
        }

        public double ComputePanelBendingStress(Curve c, double h, double E, double steps)
        {
            double step = 1 / steps;
            double MaxBending = 0;
            c.Domain = new Interval(0, 1);

            for (double i = 0; i >= 1; i+= step)
            {
                double curvRadius = c.CurvatureAt(i).Length;
                double k = 1 / curvRadius;
                double bending = E * k * h / 2;
                if (bending > MaxBending) MaxBending = bending;
            }
            return MaxBending;
        }

        public double ComputePanelShearStress(double h, double G, double t)
        {
            return h * G * t;
        }

        public double findMaxGaussianCurvatureInPanel(Surface s, double steps)
        {
            double maxGaussian = 0;
            double uStep = s.Domain(0)[1] - s.Domain(0)[0] / steps;
            double vStep = s.Domain(1)[1] - s.Domain(1)[0] / steps;
            for (double u = 0; u < 1; u += uStep)
            {
                for (double v = 0; v < 1; v += vStep)
                {
                    double gaussian = s.CurvatureAt(u, v).Gaussian;

                    if (gaussian > maxGaussian) maxGaussian = gaussian;
                }
            }
            return maxGaussian;
        }
    }
}
