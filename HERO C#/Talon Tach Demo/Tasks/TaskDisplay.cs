/**
 * Display Task, Periodically update a CTRE LCD Display with subsystem information
 * @link http://www.ctr-electronics.com/gadgeteer-display-module.html
 */
using Microsoft.SPOT;			//Fonts
using CTRE.Gadgeteer.Module;	//Display Module
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskDisplay : ILoopable
{
    /* fonts in order, smallest to largest */
    Font f0 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.small);
    Font f1 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.nina14);
    Font f2 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.NinaB);
    Font f3 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.ninabd18ppem);
    DisplayModule.LabelSprite[] _lines = new DisplayModule.LabelSprite[8];

    public TaskDisplay()
    {
        int width = 120;
        int height = 10;
        int x = 0;
        int y = 0;

        int i = 0;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, DisplayModule.Color.Cyan, x, y, width, height); y += height;
    }
       
    public void OnLoop()
    {
        /* use the various toString routines of subsystems or tasks */
        _lines[0].SetText(Platform.Subsystems.Arm.ToString());
        _lines[1].SetText(Platform.Subsystems.Wheel.ToString());
        _lines[2].SetText(Platform.Tasks.taskLowBatteryDetect.ToString());
		/* Use lines [3,7] to add extra telematry to LCD Dispaly */ 
	}

	/* ILoopables */
	public void OnStart() { }
    public void OnStop() { }
    public bool IsDone() { return false; }
}