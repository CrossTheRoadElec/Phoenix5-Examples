/**
 * Example Windows application that pings an access point that will return data.  It is intended to be used with the ESP12F Wifi Ring Buffer Example.
 * This application will send data to a designated access point using a specified protocol and attempt to receive the data back.
 * Communication type and Access Point Settings can be changed at the top of Main().
 */

#define UDP
//#define TCP

using System;
using System.Linq;
using System.Net.Sockets;
using System.Net;



namespace Windows_Ping_Example
{
    class Program
    {
        const string mode = "UDP";

        static byte[] toSend = new byte[64];
        static byte[] receivedData = new byte[64];

        static long startTicks = 0;
        static long endTicks = 0;
        static long waitTicks = 0;
        static int pingTime = 0;

        const long kTicksMs = TimeSpan.TicksPerMillisecond;

        static int lostCount = 0;
        static int timedOutCount = 0;
        static long successCount = 0;

        static bool sent = false;
        static bool received = false;

        static int minPing = 1000;
        static int maxPing = 0;


        static void Main(string[] args)
        {

            IPAddress serverAddr = IPAddress.Parse("192.168.4.1");
            IPEndPoint server = new IPEndPoint(serverAddr, 11001);

#if (UDP)
            UdpClient udpClient = new UdpClient(11000);
            udpClient.Connect(serverAddr, 11001);
            udpClient.Client.ReceiveTimeout = 2000;
#endif

#if (TCP)
            TcpClient tcpClient = new TcpClient("192.168.4.1",11001);
            tcpClient.Client.ReceiveTimeout = 2000;
            NetworkStream stream = tcpClient.GetStream();
#endif


            byte count = 0;

            while (true)
            {

                if (!sent)
                {
                    if (count++ > 255) { count = 0; }
                    populate(toSend, count);
                    sent = true;
                    startTicks = DateTime.Now.Ticks;
#if (UDP)
                    udpClient.Send(toSend, toSend.Length);
#endif
#if(TCP)
                    stream.Write(toSend, 0, toSend.Length);
#endif
                }

                if (sent && !received)
                {
                    try
                    {
#if (UDP)
                        byte[] receivedData = udpClient.Receive(ref server);
#endif
#if(TCP)
                        int bytes = stream.Read(receivedData, 0, receivedData.Length);
#endif

                        endTicks = DateTime.Now.Ticks;

                        if (receivedData.SequenceEqual(toSend))
                        {
                            pingTime = (int)((endTicks - startTicks) / kTicksMs);

                            successCount++;

                            Console.Out.Write(receivedData[0] + ": " + pingTime + " ms |   " + successCount + " Received,  " + timedOutCount + " Timed Out,  " + lostCount + " Lost.\r\n");

                            received = true;
                        }
                        else
                        {
                            lostCount++;
                            received = true;
                            Console.Out.Write("Packet " + toSend[0] + " Corrupt  |   " + successCount + " Received,  " + timedOutCount + " Timed Out,  " + lostCount + " Lost.\r\n");
                        }
                    }
#if (UDP)
                    catch (System.Net.Sockets.SocketException ex)
                    {
                        if (ex.SocketErrorCode == SocketError.TimedOut)
                        {
                            timedOutCount++;
                            received = true;
                            Console.Out.Write("Packet " + toSend[0] + " Timed Out  |   " + successCount + " Received,  " + timedOutCount + " Timed Out,  " + lostCount + " Lost.\r\n");
                        }
                    }
#endif
#if(TCP)
                    catch(System.IO.IOException ex)
                    {
                        //if (ex == SocketError.TimedOut)
                        {
                            timedOutCount++;
                            received = true;
                            Console.Out.Write("Packet " + toSend[0] + " Timed Out  |   " + successCount + " Received,  " + timedOutCount + " Timed Out,  " + lostCount + " Lost.\r\n");
                        }
                    }
#endif 
                }

                if (sent && received)
                {
                    sent = false;
                    received = false;
                }
				
				System.Threading.Thread.Sleep(10);
			}

        }

        public static void populate(byte[] array, byte value)
        {
            for (int i = 0; i < array.Length; i++)
            {
                array[i] = value;
            }
        }

    }
}
