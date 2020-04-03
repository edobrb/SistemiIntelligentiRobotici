MAX_VELOCITY = 30  --cm/s
require('vector2')

--[[ This function is executed every time you press the 'execute' button ]]
function init()
	robot.wheels.set_velocity(0,0)
end

-- UTILS
function ternary(c,t,f) if c then return t else return f end end
function math.sign(x) return ternary(x < 0, -1, ternary(x > 0, 1, 0)) end

proximityNearVector = v2(0,0)
proximityNearAlpha = 0.3
function calcNearestProximityVector()
	sum = v2(0,0)
	max = 1
	for i=2,#robot.proximity do
		if(robot.proximity[i].value > robot.proximity[max].value) then
			max = i
		end
	end
	sum = sum + v2P({angle = robot.proximity[max].angle, length = robot.proximity[max].value})
	proximityNearVector = proximityNearVector * (1 - proximityNearAlpha) + sum * proximityNearAlpha
	return proximityNearVector
end

proximityVector = v2(0,0)
proximityAlpha = 0.3
function calcProximityVector()
	sum = v2(0,0)
	for i=1,#robot.proximity do
	    sensor_angle = robot.proximity[i].angle
		sum = sum +  v2P({angle = robot.proximity[i].angle, length = robot.proximity[i].value})
	end
	proximityVector = proximityVector * (1 - proximityAlpha) + sum * proximityAlpha
	return proximityVector
end

lightVector = v2(0,0)
lightAlpha = 0.3
function calcLightingVector()
	sum = v2(0,0)
	for i=1,#robot.light do
	    sensor_angle = robot.light[i].angle
		sum = sum + v2P({angle = robot.light[i].angle, length = robot.light[i].value})
	end
	lightVector =  lightVector * (1 - lightAlpha) + sum * lightAlpha
	return lightVector
end

function linearVelocity(v)
    L = robot.wheels.axis_length
	return v2(v.length - v.angle * L / 2, v.length + v.angle * L / 2)
end

function magnitudeAbove(v, threshold)
	return ternary(v.length > threshold, v, zero_field)
end

-- Motors schemas
zero_field = v2P({angle = 0, length = 0})
function lightField(multiplier, threshold)
	result = calcLightingVector() * multiplier
	return magnitudeAbove(result, threshold) 
end
function forwardField(multiplier, threshold)
	return v2P({ angle = 0, length = 1}) * multiplier
end
function obstacleField(multiplier, threshold)
	result = -calcProximityVector() * multiplier
	return magnitudeAbove(result, threshold) 
end
function obstacleFieldCircum(multiplier, threshold)
	result = -calcProximityVector() * multiplier
	result = result.rotate(-math.sign(result.angle) * math.pi / 2)
	return magnitudeAbove(result, threshold) 
end

--[[ This function is executed at each time step It must contain the logic of your controller ]]
function step()
	fieldVctor = 
		obstacleField(3, 3) + 
		obstacleFieldCircum(1, 0.1) + 
		forwardField(0.5, 0) + 
		lightField(2, 0.1)
	
	vel = linearVelocity(fieldVctor)
	m = math.max(vel.x, vel.y)
	vel = vel / m * MAX_VELOCITY
	
	robot.wheels.set_velocity(vel.x,vel.y)
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
