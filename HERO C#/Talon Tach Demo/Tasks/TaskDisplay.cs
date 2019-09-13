/**
 * Periodically update a CTRE LCD Display.
 * @link http://www.ctr-electronics.com/gadgeteer-display-module.html
 */
using CTRE.Phoenix.Tasking;
using Microsoft.SPOT;
using System;
using Platform;
using CTRE.Gadgeteer.Module;

public class TaskDisplay : ILoopable
{
    /* fonts in order smallest to largest */
    Font f0 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.small);
    Font f1 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.nina14);
    Font f2 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.NinaB);
    Font f3 = Top.Properties.Resources.GetFont(Top.Properties.Resources.FontResources.ninabd18ppem);
    CTRE.Gadgeteer.Module.DisplayModule.LabelSprite[] _lines = new CTRE.Gadgeteer.Module.DisplayModule.LabelSprite[8];

    public TaskDisplay()
    {
        int width = 120;
        int height = 10;
        int x = 0;
        int y = 0;

        int i = 0;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;
        _lines[i++] = Hardware.displayModule.AddLabelSprite(f0, CTRE.Gadgeteer.Module.DisplayModule.Color.Cyan, x, y, width, height); y += height;

    }
       
    public void OnLoop()
    {
        /* use the various tostring routines of subsystems or tasks */
        _lines[0].SetText(Platform.Subsystems.Arm.ToString());
        _lines[1].SetText(Platform.Subsystems.Wheel.ToString());
        _lines[1].SetText(Platform.Tasks.taskLowBatteryDetect.ToString());
    }


    public void OnStart() { }
    public void OnStop() { }
    public bool IsDone() { return false; }
}