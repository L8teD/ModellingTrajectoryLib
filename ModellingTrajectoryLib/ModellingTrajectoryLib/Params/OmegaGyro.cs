using ModellingTrajectoryLib.Matrix;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ModellingTrajectoryLib.Params
{
    class OmegaGyro
    {
        public double X { get; private set; }
        public double Y { get; private set; }
        public double Z { get; private set; }
        public double E { get; private set; }
        public double N { get; private set; }
        public double H { get; private set; }
        public double X_dot { get; private set; }
        public double Y_dot { get; private set; }
        public double Z_dot { get; private set; }
        public OmegaGyro(AbsoluteOmega absOmega, OmegaEarth omegaEarth, GetMatrix getMatrix, Point point, Velocity velocity, Acceleration acceleration, EarthModel earth)
        {
            GetProjectionsNZSK(absOmega, omegaEarth);
            GetProjectionSSK(getMatrix);
            GetDot(point, velocity, acceleration, earth, omegaEarth);
        }

        private void GetProjectionsNZSK(AbsoluteOmega absOmega, OmegaEarth omegaEarth)
        {
            //OmegaGyro omegaGyro = new OmegaGyro();
            E = absOmega.E;
            N = absOmega.N + omegaEarth.N;
            H = absOmega.H + omegaEarth.H;
            //return omegaGyro;
        }
        private void GetProjectionSSK(GetMatrix getMatrix)
        {
            double[][] omega_NEH = new double[][] { new double[] { E }, new double[] { N }, new double[] { H } };
            double[][] omega_XYZ = MatrixOperations.Product(MatrixOperations.Inverted(getMatrix.C), omega_NEH);
            X = omega_XYZ[0][0];
            Y = omega_XYZ[1][0];
            Z = omega_XYZ[2][0];
        }
        private void GetDot(Point point, Velocity velocity, Acceleration acceleration, EarthModel earth, OmegaEarth omegaEarth)
        {
            X_dot = -(velocity.N_dot - velocity.H * velocity.N / earth.R1) / earth.R1;
            Y_dot = (velocity.E_dot - omegaEarth.H * velocity.N - velocity.H * velocity.E / earth.R2) / earth.R2;
            Z_dot = (omegaEarth.N * velocity.N + acceleration.E * Math.Tan(point.lat) + velocity.H * this.H +
                (velocity.E * velocity.N) / (earth.R2 * Math.Pow(Math.Cos(point.lat), 2))) / earth.R2;
        }
    }
}
