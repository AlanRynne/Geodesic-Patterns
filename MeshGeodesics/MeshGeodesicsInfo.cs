using System;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;

namespace MeshGeodesics
{
    /// <summary>
    /// Mesh geodesics info.
    /// </summary>
    public class MeshGeodesicsInfo : GH_AssemblyInfo
    {
        /// <summary>
        /// Gets the name.
        /// </summary>
        /// <value>The name.</value>
        public override string Name
        {
            get
            {
                return "MeshGeodesics Info";
            }
        }

        /// <summary>
        /// Gets the icon.
        /// </summary>
        /// <value>The icon.</value>
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }

        /// <summary>
        /// Gets the description.
        /// </summary>
        /// <value>The description.</value>
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }

        /// <summary>
        /// Gets the identifier.
        /// </summary>
        /// <value>The identifier.</value>
        public override Guid Id
        {
            get
            {
                return new Guid("e418ea81-024e-4176-924d-82d56fa6c3fc");
            }
        }

        /// <summary>
        /// Gets the name of the author.
        /// </summary>
        /// <value>The name of the author.</value>
        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        /// <summary>
        /// Gets the author contact.
        /// </summary>
        /// <value>The author contact.</value>
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
