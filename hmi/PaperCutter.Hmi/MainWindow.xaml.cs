using System.Windows;
using PaperCutter.Hmi.ViewModels;

namespace PaperCutter.Hmi
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainViewModel();
        }
    }
}
