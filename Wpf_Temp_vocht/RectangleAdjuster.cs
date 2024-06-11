using System.Windows;
using System.Windows.Shapes;

namespace Wpf_Temp_vocht
{
    public class RectangleAdjuster
    {
        public void AdjustRectangleHeight(double temperature, Rectangle temperatureRectangle)
        {
            double percentage = temperature * 2.6 * 2.9 / 100;
            double vertPos = 31 + 290 - percentage;
            temperatureRectangle.Height = percentage;
            temperatureRectangle.Margin = new Thickness(70, vertPos, 0, 0);
        }

        public void AdjustRectangleHeighthum(double humidity, Rectangle humRectangle)
        {
            double percentage = humidity * 1.55 / 100;
            double vertPos = 4 + 155 - percentage;
            humRectangle.Height = percentage;
            humRectangle.Margin = new Thickness(0, vertPos, 0, 0);
        }
    }
}