using System;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;

namespace StartDirGeodesic
{
    public class StartDirGeodesicInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "StartDirGeodesic Info";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("3dfc624b-2f1a-4689-92a7-70949a17f23d");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
