using ModellingTrajectoryLib.Errors;
using ModellingTrajectoryLib.Helper;
using ModellingTrajectoryLib.Matrix;
using ModellingTrajectoryLib.Params;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static ModellingTrajectoryLib.Types;

namespace ModellingTrajectoryLib
{
    class ModellingFunctions
    {
        double Rz = 6371000;
        double g = 9.81;
        double[] dLon;
        double[] ortDist;
        double[] ortDistAngle;
        double[] heading;
        double[] distance;
        double[] pitch;
        double[] dH;
        double rollTarget = -20;
        List<double> roll;
        double UR;
        double radiusTurn;
        double timeTurn;
        double LUR_Distance;
        double PPM_Distance;
        double[] velAbs;

        GetMatrix getMatrix = new GetMatrix();

        Point[] startedPoints;


        List<Point> currentCoords;
        List<Velocity> velocityList;
        List<AbsoluteOmega> absOmegaList;
        List<OmegaGyro> omegaGyroList;
        List<Acceleration> accelerationList;
        List<EarthModel> earthModelList;
        List<GravitationalAcceleration> gravitationalAccelerationList;
        List<OmegaEarth> omegaEarthList;

        public List<PointSet> outputPoints;
        public List<VelocitySet> outputVelocityList;
        public List<DisplayedData> outputDisplayedDataIdeal;
        public List<DisplayedData> outputDisplayedDataError;

        ErrorsModel errorsModel = new ErrorsModel();
        List<double[][]> errorsList;

        bool turnHappened = false;

        private void InitStartedData(double[] latArray, double[] lonArray, double[] altArray)
        {
            InitStartedCoords(latArray, lonArray, altArray);
            InitStartedVelocitites(latArray.Length - 1);
        }
        private void InitStartedCoords(double[] latArray, double[] lonArray, double[] altArray)
        {
            startedPoints = new Point[latArray.Length];
            for (int i = 0; i < latArray.Length; i++)
            {
                startedPoints[i] = new Point(latArray[i], lonArray[i], altArray[i]);
            }
        }
        private void InitStartedVelocitites(int length)
        {
            velAbs = new double[length];
            for (int i = 0; i < length; i++)
            {
                velAbs[i] = 900 / 3.6;
            }
        }
        private double[] MakeArray(int length)
        {
            return new double[length - 1];
        }
        private double ComputeOrtDistAngle(Point lastPoint, Point nextPoint)
        {
            return Math.Acos(Math.Sin(lastPoint.lat) * Math.Sin(nextPoint.lat) +
                Math.Cos(lastPoint.lat) * Math.Cos(nextPoint.lat) * Math.Cos(nextPoint.lon - lastPoint.lon));
        }
        private double ComputeHeading(Point lastPoint, Point nextPoint, double dLon)
        {
           return Math.Atan2(Math.Cos(nextPoint.lat) * Math.Sin(dLon),
                Math.Cos(lastPoint.lat) * Math.Sin(nextPoint.lat) - Math.Sin(lastPoint.lat) * Math.Cos(nextPoint.lat) * Math.Cos(dLon));
        }
        private void InitParamsBetweenPPM()
        {
            dH = MakeArray(startedPoints.Length);
            for (int i = 0; i < dH.Length; i++)
            {
                dH[i] = 0;
            }
            dLon = MakeArray(startedPoints.Length);
            ortDist = MakeArray(startedPoints.Length);
            ortDistAngle = MakeArray(startedPoints.Length);
            heading = MakeArray(startedPoints.Length);
            distance = MakeArray(startedPoints.Length);
            pitch = MakeArray(startedPoints.Length);
            roll = new List<double>();
            //roll = MakeArray(startedPoints.Length);
            for (int k = 0; k < startedPoints.Length - 1; k++)
            {
                ComputeParamsBetweenPPM(k);
            }
        }
        private void CheckParamsBetweenPPM(int k)
        {
            if (turnHappened)
            {
                ComputeParamsBetweenPPM(k);
                turnHappened = false;
            }
        }
        private void ComputeParamsBetweenPPM(int k)
        {
            dLon[k] = startedPoints[k + 1].lon - startedPoints[k].lon;
            ortDistAngle[k] = ComputeOrtDistAngle(startedPoints[k], startedPoints[k + 1]);
            ortDist[k] = Rz * ortDistAngle[k];
            distance[k] = Math.Sqrt(Math.Pow(ortDist[k], 2)) + dH[k];
            pitch[k] = Math.Atan2(dH[k], ortDist[k]);
            heading[k] = ComputeHeading(startedPoints[k], startedPoints[k + 1], dLon[k]);
            heading[k] += heading[k] <= 0 ? 2 * Math.PI : 0;
            heading[k] -= heading[k] >= 360 ? 2 * Math.PI : 0;
        }

        private void ComputeLUR(int k)
        {
            UR = heading[k + 1] - heading[k];
            UR -= UR >= Converter.DegToRad(180) ? Converter.DegToRad(360) : 0;
            UR += UR <= Converter.DegToRad(-180) ? Converter.DegToRad(360) : 0;

            rollTarget = UR >= 0 ? Math.Abs(rollTarget) : rollTarget;

            radiusTurn = Math.Pow(velAbs[k], 2) / (g * Math.Tan(Converter.DegToRad(rollTarget)));
            timeTurn = radiusTurn * UR / velAbs[k];
            LUR_Distance = radiusTurn * Math.Tan(0.5 * UR);
        }
        private double RecountHeading(double velocityRoute, double velocityAbs, Wind wind, double heading)
        {
            double a1 = Math.Pow(velocityRoute, 2) + Math.Pow(velocityAbs, 2) - Math.Pow(wind.speed, 2);
            double a2 = (2.0 * velocityAbs * velocityRoute);
            double a = Math.Acos(a1 / a2);
            return a + heading;
        }
        private double RecountVelocity(double velocityAbs, Wind wind, double heading)
        {
            return Math.Sqrt(Math.Pow(velocityAbs, 2) + Math.Pow(wind.speed, 2) - 2.0 * velocityAbs * wind.speed * Math.Cos(180 - heading - wind.angle));
        }
        
        public void ModellingTrajectory(double[] latArray, double[] lonArray, double[] altArray, InitErrors initErrors)
        {
            currentCoords = new List<Point>()
            {
                new Point(latArray[0], lonArray[0], altArray[0])
            };
            outputPoints = new List<PointSet>();
            outputVelocityList = new List<VelocitySet>();
            outputDisplayedDataIdeal = new List<DisplayedData>();
            outputDisplayedDataError = new List<DisplayedData>();
            velocityList = new List<Velocity>();
            absOmegaList = new List<AbsoluteOmega>();
            earthModelList = new List<EarthModel>();
            accelerationList = new List<Acceleration>();
            omegaGyroList = new List<OmegaGyro>();
            gravitationalAccelerationList = new List<GravitationalAcceleration>();
            omegaEarthList = new List<OmegaEarth>();
            errorsList = new List<double[][]>();

            InitStartedData(latArray, lonArray, altArray);
            InitParamsBetweenPPM();

            int i = 0;
            double dt = 1;
            for (int k = 0; k < startedPoints.Length - 1; k++)
            {
                CheckParamsBetweenPPM(k);

                if (k != heading.Length - 1)
                    ComputeLUR(k);
                else
                    LUR_Distance = -1;

                PPM_Distance = Rz * ortDistAngle[k];
                double PPM_Disctance_prev = PPM_Distance + 1;

                double currentVelocity = velAbs[k];
                double currentHeading = heading[k];
                while (LUR_Distance < PPM_Distance && PPM_Disctance_prev > PPM_Distance)
                {
                    if (i % 100 == 1)
                    {
                        Wind wind = Weather.Query(currentCoords[i]);
                        currentVelocity = RecountVelocity(velAbs[k], wind, heading[k]);
                        currentHeading = RecountHeading(currentVelocity, velAbs[k], wind, heading[k]);
                    }
                   

                    roll.Add(0);
                    getMatrix.CreateC(currentHeading, pitch[k], roll[i]);
                    earthModelList.Add(new EarthModel(currentCoords[i]));
                    velocityList.Add(new Velocity(currentVelocity, currentHeading, pitch[k]));
                    absOmegaList.Add(new AbsoluteOmega(velocityList[i], earthModelList[i], currentCoords[i]));
                    gravitationalAccelerationList.Add(new GravitationalAcceleration(currentCoords[i]));
                    omegaEarthList.Add(new OmegaEarth(currentCoords[i]));
                    accelerationList.Add(new Acceleration(absOmegaList[i], velocityList[i], gravitationalAccelerationList[i], omegaEarthList[i], getMatrix));
                    omegaGyroList.Add(new OmegaGyro(absOmegaList[i], omegaEarthList[i], getMatrix, currentCoords[i], velocityList[i], accelerationList[i], earthModelList[i]));
                    currentCoords.Add(Point.GetCoords(currentCoords[i], absOmegaList[i], velocityList[i], dt));

                    getMatrix.CreateM(heading[k], pitch[k]);
                    errorsModel.ModellingErrors(initErrors, currentCoords[i], omegaGyroList[i], accelerationList[i], earthModelList[i], getMatrix);
                    //SaveDataWithError(currentCoords[i], velocityList[i], errorsModel.X, currentCoords[i].lat, earthModelList[i]);
                    errorsList.Add(errorsModel.X);
                    
                    outputPoints.Add(new PointSet(currentCoords[i], errorsModel.X, currentCoords[i].lat, earthModelList[i]));
                    outputVelocityList.Add(new VelocitySet(velocityList[i], errorsModel.X));
                    
                    outputDisplayedDataIdeal.Add(new DisplayedData(outputPoints[i].InDegrees, outputVelocityList[i].Value, Converter.RadToDeg(currentHeading), 
                        Converter.RadToDeg(pitch[k]), Converter.RadToDeg(roll[i])));
                    outputDisplayedDataError.Add(new DisplayedData(outputPoints[i].ErrorInMeters, outputVelocityList[i].Error,
                        Converter.RadToDeg(errorsModel.anglesErrors[0][0]), Converter.RadToDeg(errorsModel.anglesErrors[1][0]), 
                        Converter.RadToDeg(errorsModel.anglesErrors[2][0])));
                    
                    i++;

                    PPM_Disctance_prev = PPM_Distance;
                    double ortDistAngleCurrent = ComputeOrtDistAngle(currentCoords[i], startedPoints[k + 1]);
                    PPM_Distance = ortDistAngleCurrent * Rz;
                }
                if ((k != heading.Length - 1) && (timeTurn >= 0.51))
                {
                    int timeTurnInt = (int)Math.Round(timeTurn);
                    int numberOfIterations = (int)(timeTurnInt / dt);
                    double dHeading = UR / numberOfIterations;
                    double headingTurn = heading[k];

                    turnHappened = true;

                    double dVelocityOnFullTurn = velAbs[k + 1] - velAbs[k];
                    double dVelocityOnEveryIteration = dVelocityOnFullTurn != 0 ? dVelocityOnFullTurn / numberOfIterations : 0;

                    double j = 0;
                    while (j <= timeTurnInt)
                    {
                        Wind wind = Weather.Query(currentCoords[i]);
                        currentVelocity = RecountVelocity(velAbs[k], wind, heading[k]);
                        currentHeading = RecountHeading(currentVelocity, velAbs[k], wind, heading[k]);

                        roll.Add(rollTarget);
                        double velocityValue = velocityList[i - 1].value + dVelocityOnEveryIteration;
                        double distTurn = velocityValue * dt;
                        double pitchTurn = Math.Atan2(0, distTurn);
                        headingTurn += dHeading;

                        getMatrix.CreateC(headingTurn, pitchTurn, roll[i]);
                        earthModelList.Add(new EarthModel(currentCoords[i]));
                        velocityList.Add(new Velocity(velAbs[k], headingTurn, pitchTurn));
                        absOmegaList.Add(new AbsoluteOmega(velocityList[i], earthModelList[i], currentCoords[i]));
                        gravitationalAccelerationList.Add(new GravitationalAcceleration(currentCoords[i]));
                        omegaEarthList.Add(new OmegaEarth(currentCoords[i]));
                        accelerationList.Add(new Acceleration(absOmegaList[i], velocityList[i], gravitationalAccelerationList[i], omegaEarthList[i], getMatrix));
                        omegaGyroList.Add(new OmegaGyro(absOmegaList[i], omegaEarthList[i], getMatrix, currentCoords[i], velocityList[i], accelerationList[i], earthModelList[i]));
                        currentCoords.Add(Point.GetCoords(currentCoords[i], absOmegaList[i], velocityList[i], dt));

                        errorsModel.ModellingErrors(initErrors, currentCoords[i], omegaGyroList[i], accelerationList[i], earthModelList[i], getMatrix);
                        errorsList.Add(errorsModel.X);
                        
                        outputPoints.Add(new PointSet(currentCoords[i], errorsModel.X, currentCoords[i].lat, earthModelList[i]));
                        outputVelocityList.Add(new VelocitySet(velocityList[i], errorsModel.X));
                        
                        outputDisplayedDataIdeal.Add(new DisplayedData(outputPoints[i].InDegrees, outputVelocityList[i].Value, Converter.RadToDeg(currentHeading),
                                                Converter.RadToDeg(pitch[k]), roll[i]));
                        outputDisplayedDataError.Add(new DisplayedData(outputPoints[i].ErrorInMeters, outputVelocityList[i].Error,
                            Converter.RadToDeg(errorsModel.anglesErrors[0][0]), Converter.RadToDeg(errorsModel.anglesErrors[1][0]),
                            Converter.RadToDeg(errorsModel.anglesErrors[2][0])));
                        
                        i++;
                        j += dt;
                    }
                }
            }
        }
    }
}
