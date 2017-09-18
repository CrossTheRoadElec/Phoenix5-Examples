public class LinearInterpolation
{
    private Point[] _arr;
    private int _cap = 0;
    private int _sz = 0;

    private struct Point
    {
        public float x;
        public float y;
        public Point(float x, float y)
        {
            this.x = x;
            this.y = y;
        }
    }
    private void Reserve(int cap)
    {
        if (cap < 2) { cap = 2; }
        _cap = cap;
        _arr = new Point[cap];
    }

    public LinearInterpolation(int cap)
    {
        Reserve(cap);
    }
    public LinearInterpolation(float x1, float y1, float x2, float y2)
    {
        Reserve(4);
        Insert(x1, y1);
        Insert(x2, y2);
    }

    public void Insert(float x, float y)
    {
        Point p;
        p.x = x;
        p.y = y;
        Insert(p);
    }
    public float Calculate(float x)
    {
        Point p1 = new Point(), p2 = new Point();
        Lookup(x, ref p1, ref p2);

        return Calculate(x, p1.x, p1.y, p2.x, p2.y);
    }
    public static float Calculate(float x, float x1, float y1, float x2, float y2)
    {
        float m = (y2 - y1) / (x2 - x1);

        float retval = m * (x - x1) + y1;
        return retval;
    }

    private void Insert(Point p)
    {
        if (_sz < _cap)
        {
            _arr[_sz] = p;
            ++_sz;
        }
        Sort();
    }

    
    private void Lookup(float x, ref Point first, ref Point second)
    {
        if (x < _arr[0].x)
        {
            first = _arr[0];
            second = _arr[1];
            return;
        }
        if (x > _arr[_sz - 1].x)
        {

            first = _arr[_sz - 2];
            second = _arr[_sz - 1];
            return;
        }
        int start = 0;
        int end = 1;

        int loops = _sz - 1;
        while (loops > 0)
        {
            --loops;

            if ((_arr[start].x <= x) && (x <= _arr[end].x))
            {
                first = _arr[start];
                second = _arr[end];
                return;
            }
            ++start;
            ++end;
        }
    }

    private void Sort()
    {  
        /* bubble sort for now */
        int i, j, flag = 1;    // set flag to 1 to start first pass
        Point temp;             // holding variable
        int numLength = _sz;
        for (i = 1; (i <= numLength) && (flag > 0); i++)
        {
            flag = 0;
            for (j = 0; j < (numLength - 1); j++)
            {
                if (_arr[j + 1].x < _arr[j].x)      // ascending order simply changes to <
                {
                    temp = _arr[j];             // swap elements
                    _arr[j] = _arr[j + 1];
                    _arr[j + 1] = temp;
                    flag = 1;               // indicates that a swap occurred.
                }
            }
        }
    }
}
