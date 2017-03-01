package org.usfirst.frc.team5811.robot;

public enum RobotStates{
	/*
	 * How to read: (sequence of methods [i.e. shootgear = shoot then place gear])
	 * + (location on field where interaction occurs [i.e. Boiler is at the Boiler])
	 * + (side of field, depends on alliance due to field being rotationally asymmetrical [i.e. Boiler Right is Red Alliance])
	*/ 
	
	//**************BASE FUNCTIONS**************
	
	//Baseline Only
	baseline,
	
	//Gear Base
	gearMiddle,
	
	//**************BOILER-BASED FUNCTIONS**************
	
	//Gear Only
	gearMiddleBoilerLeft,
	gearMiddleBoilerRight,
	gearBoilerLeft,
	gearBoilerLeftWhileMiddle,
	gearBoilerRight,
	gearBoilerRightWhileMiddle,
	
	//Shoot Only
	shootOnlyBoilerLeft,
	shootOnlyBoilerLeftWhileMiddle,
	shootOnlyBoilerRight,
	shootOnlyBoilerRightWhileMiddle,
	
	//Hopper Only
	hopperOnlyBoilerLeft,
	hopperOnlyBoilerLeftWhileMiddle,
	hopperOnlyBoilerRight,
	hopperOnlyBoilerRightWhileMiddle,
	
	//Gear and Shoot
	gearMiddleShootBoilerLeft, //Priority
	gearMiddleShootBoilerRight,  //Priority
	gearShootBoilerLeft,
	gearShootBoilerLeftWhileMiddle,
	gearShootBoilerRight,
	gearShootBoilerRightWhileMiddle,
	
	//Hopper then Shoot
	hopperShootBoilerLeft,
	hopperShootBoilerLeftWhileMiddle,
	hopperShootBoilerRight,
	hopperShootBoilerRightWhileMiddle,

	//Gear and Hopper
	gearMiddleHopperBoilerLeft,
	gearMiddleHopperBoilerRight,
	gearHopperBoilerLeft,
	gearHopperBoilerLeftWhileMiddle,
	gearHopperBoilerRight,
	gearHopperBoilerRightWhileMiddle,
	
	//Hopper, Shoot, Place Gear || Place Gear, Hopper, then Shoot (Hopper Shoot Gear Sequences)
	gearMiddleHopperShootBoilerLeft,
	gearMiddleHopperShootBoilerRight,
	gearHopperShootBoilerLeft,
	gearHopperShootBoilerLeftWhileMiddle,
	gearHopperShootBoilerRight,
	gearHopperShootBoilerRightWhileMiddle,

	//Shoot, Hopper, Shoot
	shootHopperShootBoilerLeft,
	shootHopperShootBoilerLeftWhileMiddle,
	shootHopperShootBoilerRight,
	shootHopperShootBoilerRightWhileMiddle,
	
	//Ultimate Autonomous
	ultimateAutoGearMiddleBoilerLeft,
	ultimateAutoGearMiddleBoilerRight,
	ultimateAutoBoilerLeft, //gear shoot hopper shoot boiler left
	ultimateAutoBoilerLeftWhileMiddle,
	ultimateAutoBoilerRight,  //gear shoot hopper shoot boiler right
	ultimateAutoBoilerRightWhileMiddle,
	
	
	//**************LOADING-BASED FUNCTIONS**************
	
	//Loading Only
	loadingOnlyLeft,
	loadingOnlyLeftWhileMiddle,
	loadingOnlyRight,
	loadingOnlyRightWhileMiddle,
	
	//Gear and Loading
	gearMiddleLoadingLeft,
	gearMiddleLoadingRight,
	gearLoadingLeft,
	gearLoadingLeftWhileMiddle,
	gearLoadingRight,
	gearLoadingRightWhileMiddle,
	
	//Hopper and Loading
	hopperLoadingLeft,
	hopperLoadingLeftWhileMiddle,
	hopperLoadingRight,
	hopperLoadingRightWhileMiddle,
	
	//Hopper and Gear
	gearMiddleHopperLoadingLeft,
	gearMiddleHopperLoadingRight,
	gearHopperLoadingLeft,
	gearHopperLoadingLeftWhileMiddle,
	gearHopperLoadingRight,
	gearHopperLoadingRightWhileMiddle,
	
	//**************************************************************
	turnMacroTest,
	noStringNoMove,
	
	test,
}