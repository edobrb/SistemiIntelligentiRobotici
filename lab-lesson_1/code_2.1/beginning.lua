-- L'idea di base è quella di calcoalre la direzione della luce rispetto a quella del robot
-- basandosi sui 24 sensori disponibili, dopodichè si ruota il robot finchè l'angolo della luce 
-- percepita è vicino al valore 0; una volta che si riscontra che la luce è pressochè in direzione
-- frontale rispetto al robot si avviano i motori a piena potenza in avanti. 

ANGLE_TRESHOLD = 0.2 --radiant
MAX_VELOCITY = 30  --cm/s

--[[ This function is executed every time you press the 'execute' button ]]
function init()
	robot.wheels.set_velocity(0,0)
end

function math.sign(x)
   if x<0 then
     return -1
   elseif x>0 then
     return 1
   else
     return 0
   end
end

--[[ This function is executed at each time step It must contain the logic of your controller ]]
function step()

    sumX = 0.0
	sumY = 0.0
	for i=1,#robot.light do
	    sensor_angle = -(-(math.pi * 2 / (#robot.light * 2)) + i * math.pi * 2 / #robot.light)
		x = robot.light[i].value * math.cos(sensor_angle)
		y = robot.light[i].value * math.sin(sensor_angle)
		sumX = sumX + x
		sumY = sumY + y
	end
	
    target_angle = math.atan2(sumY,sumX)
	light_manitude = sumX * sumX + sumY * sumY
	log("light_manitude:" .. light_manitude)
	
	LIGHT_THRESHOLD = 0.05
	
	if light_manitude > LIGHT_THRESHOLD then
		if math.abs(target_angle) > ANGLE_TRESHOLD then
			left_v = math.sign(target_angle)*MAX_VELOCITY
			right_v = -math.sign(target_angle)*MAX_VELOCITY
			robot.wheels.set_velocity(left_v,right_v)
		else
			robot.wheels.set_velocity(MAX_VELOCITY, MAX_VELOCITY)
		end
	else
		randomWalk()
	end
	
end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	robot.wheels.set_velocity(0,0)
end

function randomWalk()
	sumX = 0.0
	sumY = 0.0
	for i=1,#robot.proximity do
	    sensor_angle = -(-(math.pi * 2 / (#robot.proximity*2)) + i * math.pi * 2 / #robot.proximity)
		
		x = robot.proximity[i].value * math.cos(sensor_angle)
		y = robot.proximity[i].value * math.sin(sensor_angle)
		sumX = sumX + x
		sumY = sumY + y
	end

	obstacle_angle = math.atan2(sumY,sumX)
	left_v = robot.random.uniform(0, MAX_VELOCITY) - (math.sign(obstacle_angle)*MAX_VELOCITY)
	right_v = robot.random.uniform(0, MAX_VELOCITY) + (math.sign(obstacle_angle)*MAX_VELOCITY)
	robot.wheels.set_velocity(left_v, right_v)
end


--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
