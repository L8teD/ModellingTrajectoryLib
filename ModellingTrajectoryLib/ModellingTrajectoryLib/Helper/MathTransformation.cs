﻿using ModellingTrajectoryLib.Params;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ModellingTrajectoryLib.Helper
{
    class MathTransformation
    {
        public static double ReverseAngle(double value)
        {
            return Converter.DegToRad(360) - value;
        }
        public static void IncrementValue(ref double value, double increment)
        {
            value += increment;
        }
        public static Point SumCoordsAndErrors(Point coord, Point error)
        {
            return new Point(
                coord.lat + error.lat,
                coord.lon + error.lon,
                coord.alt + error.alt
                );
        }
    }
}
