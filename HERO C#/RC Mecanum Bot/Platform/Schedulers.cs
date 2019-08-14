using CTRE.Phoenix.Tasking;

namespace HERO_Mecanum_Drive_Example.Platform
{
    public static class Schedulers
    {
        /* the schedulers.  Minimally you will likely want one periodic scheduler to run the normal tasks.
         * Additional schedulers could be ConsecutiveSchedulers for entire autonomous movements or preconfigured manuevers.
         * Use 'public static' because these are single objects. */

        public static ConcurrentScheduler PeriodicTasks = new ConcurrentScheduler(10);
    }
}
