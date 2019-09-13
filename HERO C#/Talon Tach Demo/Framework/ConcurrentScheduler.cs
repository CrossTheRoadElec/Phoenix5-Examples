/** changes to the framework class below will be merged into Phoenix Framework */
using Microsoft.SPOT;

namespace CTRE.Phoenix.Tasking
{
    public class ConcurrentScheduler : IProcessable, ILoopable
    {
        System.Collections.ArrayList _loops = new System.Collections.ArrayList();
        System.Collections.ArrayList _enabs = new System.Collections.ArrayList();

        int _periodMs;
        PeriodicTimeout _timeout;

        public ConcurrentScheduler(int periodMs)
        {
            _periodMs = periodMs;
            _timeout = new PeriodicTimeout(periodMs);
        }
        public void Add(ILoopable newLoop, bool bEnabled = true)
        {
            foreach (var loop in _loops)
            {
                if (loop == newLoop)
                    return; /* already here */
            }
            _loops.Add(newLoop);
            _enabs.Add(bEnabled);
            /* call the start routine now, and next process call will be it's first process */
            if (bEnabled)
                Start(newLoop);
        }

        public void Start(ILoopable toStart)
        {
            for (int i = 0; i < _loops.Count; ++i)
            {
                ILoopable lp = (ILoopable)_loops[i];

                if (lp == toStart)
                {
                    lp.OnStart();
                    _enabs[i] = true;
                 
                    return;
                }
            }
            Debug.Print("CTR: Could not find object in scheduler");
        }
        public void Stop(ILoopable toStart)
        {
            for (int i = 0; i < _loops.Count; ++i)
            {
                ILoopable lp = (ILoopable)_loops[i];
                bool en = (bool)_enabs[i];

                if (lp == toStart)
                {
                    if (en == true)
                    {
                        lp.OnStop();
                        _enabs[i] = false;
                    }
                    return;
                }
            }
            Debug.Print("CTR: Could not find object in scheduler");
        }

        public void RemoveAll()
        {
            _loops.Clear();
            _enabs.Clear();
        }

        public void StartAll()
        {
            for (int i = 0; i < _loops.Count; ++i)
            {
                ILoopable lp = (ILoopable)_loops[i];

                lp.OnStart();
                _enabs[i] = true;
            }
        }
        public void StopAll()
        {
            for (int i = 0; i < _loops.Count; ++i)
            {
                ILoopable lp = (ILoopable)_loops[i];

                lp.OnStop();
                _enabs[i] = false;
            }
        }
        public void Process()
        {
            if (_timeout.Process())
            {
                for (int i = 0; i < _loops.Count; ++i)
                {
                    ILoopable lp = (ILoopable)_loops[i];
                    bool en = (bool)_enabs[i];

                    if (en)
                    {
                        lp.OnLoop();
                    }
                    else
                    {
                        /* this loopable is turned off, don't call it */
                    }
                }
            }
        }
        //--- Loopable ---/
        public void OnStart()
        {
            StartAll();
        }

        public void OnLoop()
        {
            Process();
        }

        public bool IsDone()
        {
            /* TODO poll all the tasks, if all tasks are enabled and done, then return true */
            return false;
        }

        public void OnStop()
        {
            StopAll();
        }
    }
}