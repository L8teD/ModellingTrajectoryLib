using ModellingTrajectoryLib.Helper;
using ModellingTrajectoryLib.Matrix;
using ModellingTrajectoryLib.Params;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static ModellingTrajectoryLib.Types;

namespace ModellingTrajectoryLib.Errors
{
    class ErrorsModel
    {
        double[][] orientationAngles = new double[3][] { new double[1], new double[1], new double[1]};

        private double[][] angles_Dot = MatrixOperations.Zeros(3, 1);
        private double[][] X_Dot = MatrixOperations.Zeros(4, 1);

        public double[][] anglesErrors;
        public double[][] X;

        private void Model(InitErrors initErrors, Acceleration acceleration, GetMatrix getMatrix, OmegaGyro omegaGyro, EarthModel earthModel)
        {
            
            double[][] orientationErrors = MatrixOperations.Product(MatrixOperations.Inverted(getMatrix.M), Converter.DegToRad(anglesErrors));

            orientationAngles[0][0] = orientationErrors[0][0] + angles_Dot[0][0];
            orientationAngles[1][0] = orientationErrors[1][0] + angles_Dot[1][0];
            orientationAngles[2][0] = orientationErrors[2][0] + angles_Dot[2][0];

            double[][] accelerationArray = new double[][]
            {
                new double[] { acceleration.X },
                new double[] { acceleration.Y },
                new double[] { acceleration.Z },
                new double[] { 0 }
            };
            double[][] accelerationIncrementArray = new double[][] 
            { 
                new double[] { 0 },
                new double[] { initErrors.accelerationError },
                new double[] { 0 }, 
                new double[] { initErrors.accelerationError }
            };
            double[][] gyroIncrementArray = new double[][]
            {
                new double[] {initErrors.gyroError},
                new double[] {initErrors.gyroError},
                new double[] {initErrors.gyroError}
            };

            double[][] vectorStateInVerticalChannel = new double[][]
            {
                new double[] { initErrors.sateliteErrorCoord },
                new double[] { initErrors.sateliteErrorVelocity },
                new double[] { 0 },
                new double[] { 0 }
            };
            CreateErrorMatrixies(ref getMatrix, omegaGyro, earthModel);
            angles_Dot = MatrixOperations.Sum(MatrixOperations.Product(getMatrix.MatrixOrientation, orientationAngles),gyroIncrementArray);

            double[][] mat1_X = MatrixOperations.Product(getMatrix.Matrix1, X);
            double[][] mat2_ACC = MatrixOperations.Product(getMatrix.Matrix2, accelerationArray);
            double[][] mat3_X_Vert = MatrixOperations.Product(getMatrix.Matrix3, vectorStateInVerticalChannel);
            X_Dot = MatrixOperations.Sum(MatrixOperations.Sum(MatrixOperations.Sum(mat1_X,
                mat2_ACC), accelerationIncrementArray), mat3_X_Vert);
        }
        private void CreateErrorMatrixies(ref GetMatrix getMatrix, OmegaGyro omegaGyro, EarthModel earthModel)
        {
            getMatrix.CreateMatrix1(omegaGyro, earthModel);
            getMatrix.CreateMatrix2(orientationAngles[0][0], orientationAngles[1][0], orientationAngles[2][0]);
            getMatrix.CreateMatrix3(omegaGyro);
            getMatrix.CreateMatrixOrientation(omegaGyro);

        }
        public void ModellingErrors(InitErrors initErrors, Point point, OmegaGyro omegaGyro, Acceleration acceleration, EarthModel earthModel, GetMatrix getMatrix)
        {
            if (X == null || anglesErrors == null)
            {
                InitX(initErrors, point, omegaGyro);
                InintAnglesError(initErrors);
            }
               
            Model(initErrors, acceleration, getMatrix, omegaGyro, earthModel);
            IncrementX();
            IcrementAngle();
        }
        private void IncrementX()
        {
            MathTransformation.IncrementValue(ref X[0][0], X_Dot[0][0]);
            MathTransformation.IncrementValue(ref X[1][0], X_Dot[1][0]);
            MathTransformation.IncrementValue(ref X[2][0], X_Dot[2][0]);
            MathTransformation.IncrementValue(ref X[3][0], X_Dot[3][0]);
        }
        private void IcrementAngle()
        {
            MathTransformation.IncrementValue(ref anglesErrors[0][0], angles_Dot[0][0]);
            MathTransformation.IncrementValue(ref anglesErrors[1][0], angles_Dot[1][0]);
            MathTransformation.IncrementValue(ref anglesErrors[2][0], angles_Dot[2][0]);
        }
        private void InitX(InitErrors initErrors, Point point, OmegaGyro omegaGyro)
        {
            X = MatrixOperations.Zeros(4, 1);

            X[0][0] = initErrors.coordAccuracy * Math.Cos(point.lat);
            X[2][0] = initErrors.coordAccuracy;
            X[1][0] = initErrors.velocityAccuracy + omegaGyro.E * Math.Tan(point.lat) * X[0][0] + omegaGyro.H * X[2][0];
            X[3][0] = initErrors.velocityAccuracy;
        }
        private void InintAnglesError(InitErrors initErrors)
        {
            anglesErrors = new double[][] {
                new double[] { Converter.DegToRad(initErrors.headingAccuracy) },
                new double[] { Converter.DegToRad(initErrors.pithAccuracy) },
                new double[] { Converter.DegToRad(initErrors.rollAccuracy) } };
        }

    }
}
