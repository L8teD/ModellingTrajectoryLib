using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ModellingTrajectoryLib.Params
{
    public class Point
    {
        public double lat { get; private set; }
        public double lon { get; private set; }
        public double alt { get; private set; }
        public Point(double _lat, double _lon, double _alt)
        {
            lat = _lat;
            lon = _lon;
            alt = _alt;
        }
        private static double[] GetCoordsIncrement(Point point, AbsoluteOmega absOmega, Velocity velocity)
        {
            return new double[] { absOmega.E, absOmega.N / Math.Cos(point.lat), velocity.H };
        }
        public static Point GetCoords(Point pointPrev, AbsoluteOmega absOmega, Velocity velocity, double dt)
        {
            double[] increments = GetCoordsIncrement(pointPrev, absOmega, velocity);
            double lat = pointPrev.lat + increments[0] * dt;
            double lon = pointPrev.lon + increments[1] * dt;
            double alt = pointPrev.alt + increments[2] * dt;
            return new Point(lat, lon, alt);
        }

    }
}
