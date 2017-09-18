/* 4x basic mecanum */
using System;
using Microsoft.SPOT;
using CTRE.Mechanical;
using CTRE.MotorControllers;

namespace CTRE.Drive
{
    public class Mecanum : IDrivetrain
    {
        Gearbox _1;
        Gearbox _2;
        Gearbox _3;
        Gearbox _4;

        float _forward;
        float _strafe;
        float _turn;

        /** Contstructor that takes 4 SimpleMotorcontrollers */
        public Mecanum(IMotorController m1, IMotorController m2, IMotorController m3, IMotorController m4)
        {
            //GroupMotorControllers.Register(m1); // commented out to compile

            /* Creat 4 single motor gearboxes */
            Gearbox temp1 = new Gearbox(m1);
            Gearbox temp2 = new Gearbox(m2);
            Gearbox temp3 = new Gearbox(m3);
            Gearbox temp4 = new Gearbox(m4);

            _1 = temp1;
            _2 = temp2;
            _3 = temp3;
            _4 = temp4;
        }

        /** Contstructor that takes 4 Gearboxes */
        public Mecanum(Gearbox m1, Gearbox m2, Gearbox m3, Gearbox m4)
        {
            // GroupMotorControllers.Register(m1.GetMaster());  // commented out to compile

            _1 = m1;
            _2 = m2;
            _3 = m3;
            _4 = m4;
        }

        /**
         * Uses forward and turn (Arcade drive)
         * 
         * @param   forward     Y direction of robot
         * @param   turn        twist of the robot
         */
        public void Set(Styles.Basic mode, float forward, float turn)
        {
            Drive(mode, forward, 0, turn);
        }

        /**
         * Uses forward, strafe, and turn (Mecanum drive)
         * 
         * @param   forward     Y direction of robot
         * @param   strafe      X direction of robot
         * @param   turn        twist of the robot (arch)
         */
        public void Set(Styles.Basic  mode, float forward, float strafe, float turn)
        {
            Drive(mode, forward, strafe, turn);
        }

        public void SetVoltageRampRate(float rampRate)
        {
            _1.SetVoltageRampRate(rampRate);
            _2.SetVoltageRampRate(rampRate);
            _3.SetVoltageRampRate(rampRate);
            _4.SetVoltageRampRate(rampRate);
        }

        public void SetVoltageCompensationRampRate(float rampRate)
        {
            _1.SetVoltageCompensationRampRate(rampRate);
            _2.SetVoltageCompensationRampRate(rampRate);
            _3.SetVoltageCompensationRampRate(rampRate);
            _4.SetVoltageCompensationRampRate(rampRate);
        }

        public void ConfigPeakPercentOutputVoltage(float forwardVoltage, float reverseVoltage)
        {
            _1.ConfigPeakOutputVoltage(forwardVoltage, reverseVoltage);
            _2.ConfigPeakOutputVoltage(forwardVoltage, reverseVoltage);
            _3.ConfigPeakOutputVoltage(forwardVoltage, reverseVoltage);
            _4.ConfigPeakOutputVoltage(forwardVoltage, reverseVoltage);
        }

        public void ConfigNominalPercentOutputVoltage(float forwardVoltage, float reverseVoltage)
        {
            _1.ConfigNominalOutputVoltage(forwardVoltage, reverseVoltage);
            _2.ConfigNominalOutputVoltage(forwardVoltage, reverseVoltage);
            _3.ConfigNominalOutputVoltage(forwardVoltage, reverseVoltage);
            _4.ConfigNominalOutputVoltage(forwardVoltage, reverseVoltage);
        }

        public float Forward { get { return _forward; } }

        public float Strafe { get { return _strafe; } }

        public float Turn { get { return _turn; } }

        private void Drive(Styles.Basic mode, float forward, float strafe, float turn)
        {
            /* save values for posterity */
            _forward = forward;
            _strafe = strafe;
            _turn = turn;

            float leftFrnt_throt = (forward + strafe + turn); // left front moves positive for forward, strafe-right, turn-right
            float leftRear_throt = (forward - strafe + turn); // left rear moves positive for forward, strafe-left, turn-right
            float rghtFrnt_throt = (forward - strafe - turn); // right front moves positive for forward, strafe-left, turn-left
            float rghtRear_throt = (forward + strafe - turn); // right rear moves positive for forward, strafe-right, turn-left

            /* Set control mode */
            if(mode == Styles.Basic.PercentOutput)
            {
                _1.SetControlMode(BasicControlMode.kPercentVbus);
                _2.SetControlMode(BasicControlMode.kPercentVbus);
                _3.SetControlMode(BasicControlMode.kPercentVbus);
                _4.SetControlMode(BasicControlMode.kPercentVbus);
            }
            else if(mode == Styles.Basic.Voltage)
            {
                _1.SetControlMode(BasicControlMode.kVoltage);
                _2.SetControlMode(BasicControlMode.kVoltage);
                _3.SetControlMode(BasicControlMode.kVoltage);
                _4.SetControlMode(BasicControlMode.kVoltage);
            }

            /* Set motors */
            _1.Set(leftFrnt_throt);
            _2.Set(leftRear_throt);
            _3.Set(rghtFrnt_throt);
            _4.Set(rghtRear_throt);
        }
    }
}