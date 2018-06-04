/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE.Phoenix.Tasking;
using Platform;

public class TaskPWMmotorController : ILoopable
{
    float _percentOut;
    bool _running; //!< Track if we are running so TaskMainLoop can keep "starting" this task with no extra init work.

    public void OnLoop()
    {
        /* just grab three axis and direct control the components */
        float axis = Hardware.gamepad.GetAxis(Constants.GamePadAxis_y);
        /* scale to typical pwm withds */
        float pulseUs = CTRE.Phoenix.LinearInterpolation.Calculate(axis, -1, 1000f, +1, 2000f); /* [-1,+1] => [1000,2000]us */
        /* scale to period */
        float periodUs = 4200; // hardcoded for now, this will be settable in future firmware update.
        _percentOut = pulseUs / periodUs;
        /* set it */
        Hardware.canifier.SetPWMOutput((uint)Constants.kMotorControllerCh, _percentOut);
    }

    public override string ToString()
    {
        return "TaskPWMmotorController:";
    }

    public void OnStart()
    {
        /* if we are already running, nothing to do */
        if (_running) { return; }

        /* start transmitting neutral */
        _percentOut = 0;
        Hardware.canifier.SetPWMOutput((uint)Constants.kMotorControllerCh, 0);
        Hardware.canifier.EnablePWMOutput((int)Constants.kMotorControllerCh, true);

        /* okay task is now running */
        _running = true;
    }
    public void OnStop()
    {
        /* stop transmitting PWM */
        Hardware.canifier.EnablePWMOutput((int)Constants.kMotorControllerCh, false);

        /* task has stopped, take note */
        _running = false;
    }
    public bool IsDone() { return false; }
}
