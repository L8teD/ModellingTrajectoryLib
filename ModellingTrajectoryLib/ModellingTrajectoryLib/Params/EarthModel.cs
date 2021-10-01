using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ModellingTrajectoryLib.Params
{
    public class EarthModel
    {
        protected static double g_e = 9.78049;
        protected static double q = 0.00346775;
        protected static double betta_g = 0.0053171;
        protected static double betta_g1 = 71e-7;

        protected static double a = 6378245;
        protected static double b = 6356863;
        protected static double e = Math.Sqrt(Math.Pow(a, 2) - Math.Pow(b, 2)) / a;

        internal double omegaEarth = 7.29e-5;

        internal double shulerFrequency = 1.25e-3;

        internal double R1 { get; set; }
        internal double R2 { get; set; }
        public EarthModel(Point point)
        {
            ComputeRadiusInCurrentPoint(point);
        }
        private void ComputeRadiusInCurrentPoint(Point point)
        {
            R1 = a * (1 - Math.Pow(e, 2)) / Math.Sqrt(Math.Pow(1 - Math.Pow(e, 2) * Math.Sin(point.lat),3)) + point.alt;
            R2 = a / Math.Sqrt(1 - Math.Pow(e, 2) * Math.Sin(point.lat)) + point.alt;
        }
    }
}
