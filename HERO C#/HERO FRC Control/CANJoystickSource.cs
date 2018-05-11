using System;
using CTRE.Phoenix.Controller;

namespace CTRE.Phoenix.Controller
{
	class CANJoystickSource : IGameControllerValuesProvider
	{
		/* See the Project README or roboRIO example for
		 * an explanation of the arbID.  */
		const uint baseArbId = 0x1E081400;

		private int[] _gpInts = new int[2];
		private float[] _gpFlts = new float[6];
		int _updateCnt = 0;
		uint _deviceNumber = 0;

		public void SetRef(GameController reference, uint idx){}

		/** 
         * Interface for anything that provides gamepad/joystick values (could be from a host pc or from USB attached gamepad). 
         * @return  Negative If values could not be retrieved or values are very old.  toFill is cleared.
         *          Positive if values are updated. toFill is filled in.
         */

		private int SyncGet(ref GameControllerValues toFill, uint numDwords)
		{
			/* always get latest data for now */
			_updateCnt = 0;

			//TODO: Implement CTRE.Phoenix.Controller.GetCANJoy()
			int ret = GetCANJoy(ref _updateCnt, _gpInts, (uint)numDwords, _gpFlts, (uint)_gpFlts.Length);

			if (ret < 0)
			{
				/* negative error code means data is unreliable */
				if (toFill != null)
					toFill.Copy(ZeroGameControllerValues);
				/* on the next poll, always get latest */
				_updateCnt = 0;
			}
			else
			{
				/* new data, copy it over */
				if (toFill != null)
				{
					toFill.axes[0] = _gpFlts[0];
					toFill.axes[1] = _gpFlts[1];
					toFill.axes[2] = _gpFlts[2];
					toFill.axes[3] = _gpFlts[3];
					toFill.axes[4] = _gpFlts[4];
					toFill.axes[5] = _gpFlts[5];
					toFill.btns = (uint)_gpInts[0];
					toFill.pov = _gpInts[1];
				}
			}
			return ret;
		}

		public int Sync(ref GameControllerValues toFill, uint rumbleL, uint rumbleR, uint ledCode, uint controlFlags, uint idx)
		{
			/* This doesn't set any values right now.
			 * Need to implement control frame from
			 * HERO --> RIO if we want to use these.
			 **/

			/* save commands */
			//_gpInts[20] = (int)rumbleL;
			//_gpInts[21] = (int)rumbleR;
			//_gpInts[22] = (int)ledCode;
			//_gpInts[23] = (int)controlFlags;

			/* set the device mask bits, these are global */
			//_gpInts[16] = (int)_maskBits;

			return SyncGet(ref toFill, (uint)_gpInts.Length);
		}

		public int Get(ref GameControllerValues toFill, uint idx)
		{
			/* set the device mask bits, these are global */
			//_gpInts[16] = (int)_maskBits;

			const uint numDwords = 2; /* only send params to be read */

			return SyncGet(ref toFill, numDwords);
		}

		public CANJoystickSource(uint deviceNumber)
		{
			_deviceNumber = deviceNumber;
		}

		private int GetCANJoy(ref int updateCount, int[] ints, uint numInts, float[] floats, uint numFloats)
		{
			uint length = 8;
			ulong _cache = 0;
			int retval = CTRE.Native.CAN.Receive(baseArbId | _deviceNumber, ref _cache, ref length);

			//Don't bother updating if we don't have data or if we can't hold it.
			if(retval >= 0 && numInts >= 2 && numFloats >= 6)
			{
				int param = 0;
				param = (byte)(_cache & 0xFF);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[0] = param / 127f;
				param = (byte)(_cache >> 0x08);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[1] = param / 127f;
				param = (byte)(_cache >> 0x10);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[2] = param / 127f;
				param = (byte)(_cache >> 0x18);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[3] = param / 127f;
				param = (byte)(_cache >> 0x20);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[4] = param / 127f;
				param = (byte)(_cache >> 0x28);
				param <<= (32 - 8); /* sign extend */
				param >>= (32 - 8); /* sign extend */
				floats[5] = param / 127f;

				int btns = 0;
				btns = (byte)((_cache >> 0x38) & 0xFF);
				btns <<= 4;
				btns |= (byte)((_cache >> 0x30) & 0xF);
				ints[0] = btns;

				param = (byte)((_cache >> 0x30 + 4) & 0xF);
				param <<= (32 - 4); /* sign extend */
				param >>= (32 - 4); /* sign extend */
				ints[1] = param;
			}

			return retval;
		}

		private static GameControllerValues ZeroGameControllerValues = new GameControllerValues();
	}
}
