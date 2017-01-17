**Definitions:**

	Agitator (keeps balls from being stuck)
	Block Piston (stop balls from flying)
	Elevator (lifts balls up from floor)

-------------------------------------------------------------------------------------

**Turn correction:**

(assuming trying to find 0)

If angle <= 180
	turn left motor backwards and right motor forwards

If angle is > -180
	turn right motor backwards and left motor forwards

if angle is 0
	stop both motors when that is true

-------------------------------------------------------------------------------------


**Default:**

	Agitator: On
	block piston: blocking
	elevator: Off

**Intake:**

	Agitator: Off 
	block piston: blocking
	elevator: On

**Shooting:**

	Agitator: off
	block piston: not blocking
	elevator: On
	
**Climbing:**

	Agitator: Off
	block piston: not blocking
	elevator: Off    
