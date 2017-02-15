/*
 * TeleopHelper.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: Adam
 */

#include <TeleopHelper.h>
#include <WPILib.h>

void Drive(Spark *FL, Spark *FR, TalonSRX *BR, TalonSRX *BL){
	FL->Set(0.1);
	FR->Set(0.1);
}


