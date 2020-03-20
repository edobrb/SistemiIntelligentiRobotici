-- [[ Stesso ragionamento dell'esercizio 2.1: una volta percepito un ostacolo si controllano i motori per far si di voltare dalla parte opposta dell'ostacolo]]

-- Put your global variables here

ANGLE_TRESHOLD = 0.2 --radiant
MAX_VELOCITY = 30  --cm/s



--[[ This function is executed every time you press the 'execute' button ]]
function init()
	left_v = robot.random.uniform(0,MAX_VELOCITY)
	right_v = robot.random.uniform(0,MAX_VELOCITY)
	robot.wheels.set_velocity(left_v,right_v)
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
	for i=1,#robot.proximity do
	    sensor_angle = -(-(math.pi * 2 / (#robot.proximity*2)) + i * math.pi * 2 / #robot.proximity)
		
		x = robot.proximity[i].value * math.cos(sensor_angle)
		y = robot.proximity[i].value * math.sin(sensor_angle)
		sumX = sumX + x
		sumY = sumY + y
		
	end
	log("X: " .. sumX)
	log("Y: " .. sumY)
    obstacle_angle = math.atan2(sumY,sumX)
	log("obstacle angle:" .. obstacle_angle)
	

	left_v = robot.random.uniform(0, MAX_VELOCITY) - (math.sign(obstacle_angle)*MAX_VELOCITY)
	right_v = robot.random.uniform(0, MAX_VELOCITY) + (math.sign(obstacle_angle)*MAX_VELOCITY)
	robot.wheels.set_velocity(left_v, right_v)

end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	robot.wheels.set_velocity(0,0)
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
