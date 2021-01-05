/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/*
 Description: 	READ THIS FIRST
 
 This example demonstrates how to best take advantage of the absolute
 pulse width encoded signal from the CTRE Magnetic Encoder.
 
 THIS IS NOT A CLOSED-LOOP EXAMPLE.  Instead see the motion magic GitHub example.
 
 In such applications, a mechanism rotates the mag encoder with a range of < 360 deg.
 This is necessary when tracking the position of an elevator, arm, and other mechanisms 
 that do not spin continuously, i.e. have hard limits. 
 
 In such circumstances, the magnet may wrap from the 359' position to the 0' in the 
 mechanism range depending on magnet orientation.  Two examples are displayed below.
 The shaded region represents the portion of the magnetic encoder used in your mechanism.
  
 The top of the circle | mark represents the overflow from 359' to 0'.
 Note when using a CTRE Magnetic encoder, 360' => 4096 units.
 
                  Scenario 1                      Scenario 2 
               From 10' to 350'                 From 100' to 80'
               No discontinuity.              Discontinuity present. 

                359' | 0'                        359' | 0'
                     |                                |                  
                     |                                |                  
        (350')    %@@@@%    (10')                  %@@@@%                
             *@@@         @@*                   @@@@@@@@@@@@@            
           @@@@@@&       &@@@@@              @@@@@@@@@@@@@@@@@@#  (80')  
         .@@@@@@@@      @@@@@@@@.          #@@@@@@@@@@@@@@@@@@@@@.       
         @@@@@@@@@@    #@@@@@@@@@         #@@@@@@@@@@@@@@@@@@#   #.      
        @@@@@@@@@@@@  #@@@@@@@@@@@       ,@@@@@@@@@@@@@@@@@*      @      
        @@@@@@@@@@@@**@@@@@@@@@@@@      ,@@@@@@@@@@@@@@@@         ,%  
        @@@@@@@@@@@@@@@@@@@@@@@@@@      ,@@@@@@@@@@@@@@            @     
        @@@@@@@@@@@@@@@@@@@@@@@@@@       @@@@@@@@@@@@@@@@@#       ,%     
        .@@@@@@@@@@@@@@@@@@@@@@@@.        @@@@@@@@@@@@@@@@@@@@*   @      
         *@@@@@@@@@@@@@@@@@@@@@@*          @@@@@@@@@@@@@@@@@@@@@@@  (100')
           @@@@@@@@@@@@@@@@@@@@             @@@@@@@@@@@@@@@@@@@@@        
             &@@@@@@@@@@@@@@&                 @@@@@@@@@@@@@@@@@          
                 *@@@@@@*                        #@@@@@@@@@*            

 In scenario 1, the pulse width sensor (with overflows removed) has one hard limit at 10'
 and another hard limit at 350' with no discontinuities in the travel (shaded part).  
 This also means the travel is also 340', with the remaining 20' not used.
 
 In scenario 2, the pulse width sensor (with overflows removed) has one hard limit at 80'
 and another hard limit at 100' with a discontinuity in the travel (shaded part).
 This also means the travel is also 340', with the remaining 20' not used.
 
 When the magnet is in scenario 2, an offset can be applied to produce an absolute pulse width 
 measurement with no discontinuities.  
 This can be done by subtracting the middle of the unused part.  In scenario 2 that would 
 be (80' + 100')/2 or 90'.  This also makes the middle of the unused part the new discontinuity.

                    Scenario 2 (with - 90 offset)
                Discontinuity moved to unshaded area 		
               
                           | (0' => 270')
                           |                                                           
                           |                                                           
                        %@@@@%                           
                     @@@@@@@@@@@@@            
                  @@@@@@@@@@@@@@@@@@#  (80' => 350')    
                #@@@@@@@@@@@@@@@@@@@@@.       
               #@@@@@@@@@@@@@@@@@@#   #.      
              ,@@@@@@@@@@@@@@@@@*      @      
             ,@@@@@@@@@@@@@@@@         ,%     
             ,@@@@@@@@@@@@@@            @     
              @@@@@@@@@@@@@@@@@#       ,%     
               @@@@@@@@@@@@@@@@@@@@*   @      
                @@@@@@@@@@@@@@@@@@@@@@@  (100' => 10') 
                 @@@@@@@@@@@@@@@@@@@@@        
                   @@@@@@@@@@@@@@@@@          
                      #@@@@@@@@@*   

 * Test procedure
 * [1] Adjust Talon device ID and Gamepad button assignment
 * [2] Deploy this application
 * [3] Manually move mechanism from hard limit to hard limit.
 * Note the pulseWidPos value in the console output.
 * Note if there is a wrap (4096 => 0 or 0 => 4096).
 * [4] Record measured sensor hard limit values during [3] at kBookEnd defines
 * [5] Record if sensor discontinuity was observed during [3] at kDiscontinuityPresent.
 * [6] Re-deploy and confirm selSenPos is continuous and maintains value after power cycles (persistent). 
 *
 * Controls:
 * Button 1: When button is pressed, seed the quadrature register. You can do this once 
 * 	on boot or during teleop/auton init. If you power cycle the Talon, press the button 
 * 	to confirm it's position is restored.
 * 
 * Supported Version:
 * - Talon SRX: 4.0
 * - Victor SPX: 4.0
 * - Pigeon IMU: 4.0
 * - CANifier: 4.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
	/** Hardware */
	TalonSRX _talon = new WPI_TalonSRX(2);
	Joystick _joy = new Joystick(0);

    /* Nonzero to block the config until success, zero to skip checking */
    final int kTimeoutMs = 30;
	
    /**
	 * If the measured travel has a discontinuity, Note the extremities or
	 * "book ends" of the travel.
	 */
	final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */

	/**
	 * This function is called once on roboRIO bootup
	 * Select the quadrature/mag encoder relative sensor
	 */
	public void robotInit() {
		/* Factory Default Hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();

		/* Seed quadrature to be absolute and continuous */
		initQuadrature();
		
		/* Configure Selected Sensor for Talon */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	// Feedback
											0, 											// PID ID
											kTimeoutMs);								// Timeout
	}

	/**
	 * Get the selected sensor register and print it 
	 */
	public void disabledPeriodic() {
		/**
		 * When button is pressed, seed the quadrature register. You can do this
		 * once on boot or during teleop/auton init. If you power cycle the 
		 * Talon, press the button to confirm it's position is restored.
		 */
		if (_joy.getRawButton(1)) {
			initQuadrature();
		}

		/**
		 * Quadrature is selected for soft-lim/closed-loop/etc. initQuadrature()
		 * will initialize quad to become absolute by using PWD
		 */
		int selSenPos = _talon.getSelectedSensorPosition(0);
		int pulseWidthWithoutOverflows = 
				_talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;

		/**
		 * Display how we've adjusted PWM to produce a QUAD signal that is
		 * absolute and continuous. Show in sensor units and in rotation
		 * degrees.
		 */
		System.out.print("pulseWidPos:" + pulseWidthWithoutOverflows +
						 "   =>    " + "selSenPos:" + selSenPos);
		System.out.print("      ");
		System.out.print("pulseWidDeg:" + ToDeg(pulseWidthWithoutOverflows) +
						 "   =>    " + "selSenDeg:" + ToDeg(selSenPos));
		System.out.println();
	}

	/**
	 * Seed the quadrature position to become absolute. This routine also
	 * ensures the travel is continuous.
	 */
	public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = _talon.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (kDiscontinuityPresent) {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		_talon.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
	}

	/**
	 * @param units CTRE mag encoder sensor units 
	 * @return degrees rounded to tenths.
	 */
	String ToDeg(int units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return "" + deg;
	}
}
