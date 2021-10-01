using ModellingTrajectoryLib.Helper;
using ModellingTrajectoryLib.Params;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static ModellingTrajectoryLib.Types;

namespace ModellingTrajectoryLib
{
    public class Modelling
    {
        ModellingFunctions Model = new ModellingFunctions();
        public List<PointSet> points { get; private set; }
        public List<VelocitySet> velocities { get; private set; }
        public List<DisplayedData> displayedDatasIdeal { get; private set; }
        public List<DisplayedData> displayedDatasError { get; private set; }
        public Modelling(double[] _latArray, double[] _lonArray , double[] _altArray, InitErrors initErrors)
        {
            double[] inputLatArray = Converter.DegToRad(_latArray);
            double[] inputLonArray = Converter.DegToRad(_lonArray);
            double[] inputAltArray = _altArray;
            Model.ModellingTrajectory(inputLatArray, inputLonArray, inputAltArray, initErrors);
            //points = Converter.RadToDeg(Model.returnedPoints);
            points = Model.outputPoints;
            velocities = Model.outputVelocityList;
            displayedDatasIdeal = Model.outputDisplayedDataIdeal;
            displayedDatasError = Model.outputDisplayedDataError;
        }
    }
}
