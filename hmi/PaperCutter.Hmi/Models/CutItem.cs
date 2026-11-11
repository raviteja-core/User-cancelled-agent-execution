using CommunityToolkit.Mvvm.ComponentModel;

namespace PaperCutter.Hmi.Models
{
    public partial class CutItem : ObservableObject
    {
        [ObservableProperty]
        private int _index;

        [ObservableProperty]
        private double _positionMm;

        [ObservableProperty]
        private string _status = "Pending";

        public uint PositionUnits => (uint)System.Math.Max(0, System.Math.Round(PositionMm * 100.0));

        public CutItem(int index, double positionMm)
        {
            _index = index;
            _positionMm = positionMm;
        }
    }
}
