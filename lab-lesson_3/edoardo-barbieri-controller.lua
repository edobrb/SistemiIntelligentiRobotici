MAX_VELOCITY = 30  --cm/s
require('vector2')
AXIS_L = 0
--[[ This function is executed every time you press the 'execute' button ]]
function init()
	robot.wheels.set_velocity(0,0)
	AXIS_L = robot.wheels.axis_length
end

-- UTILS
function ternary(c,t,f) if c then return t else return f end end
function math.sign(x) return ternary(x < 0, -1, ternary(x > 0, 1, 0)) end

proximityMaxVector = v2(0,0)
proximityAvgVector = v2(0,0)
proximityMaxAlpha = 0.5
proximityAvgAlpha = 0.5
function calcMaxProximityVector()
	sum = v2(0,0)
	max = 1
	for i=2,#robot.proximity do
		if(robot.proximity[i].value > robot.proximity[max].value) then
			max = i
		end
	end
	sum = sum + v2P({angle = robot.proximity[max].angle, length = robot.proximity[max].value})
	proximityMaxVector = proximityMaxVector * (1 - proximityMaxAlpha) + sum * proximityMaxAlpha
	return proximityMaxVector
end
function calcAvgProximityVector()
	sum = v2(0,0)
	for i=1,#robot.proximity do
	    sensor_angle = robot.proximity[i].angle
		sum = sum +  v2P({angle = robot.proximity[i].angle, length = robot.proximity[i].value})
	end
	proximityAvgVector = proximityAvgVector * (1 - proximityAvgAlpha) + sum * proximityAvgAlpha
	return proximityAvgVector
end

lightNearVector = v2(0,0)
lightNearAlpha = 0.5
function calcMaxLightingVector()
	sum = v2(0,0)
	max = 1
	for i=2,#robot.light do
		if(robot.light[i].value > robot.light[max].value) then
			max = i
		end
	end
	sum = sum + v2P({angle = robot.light[max].angle, length = robot.light[max].value})
	lightNearVector = lightNearVector * (1 - lightNearAlpha) + sum * lightNearAlpha
	return lightNearVector
end
lightVector = v2(0,0)
lightAlpha = 0.5
function calcAvgLightingVector()
	sum = v2(0,0)
	for i=1,#robot.light do
	    sensor_angle = robot.light[i].angle
		sum = sum + v2P({angle = robot.light[i].angle, length = robot.light[i].value})
	end
	lightVector =  lightVector * (1 - lightAlpha) + sum * lightAlpha
	return lightVector
end

function linearVelocity(v)
	return v2(v.length - v.angle * AXIS_L / 2, v.length + v.angle * AXIS_L / 2)
end

function fieldThreshold(v, threshold)
	return ternary(v.length > threshold, v, zero_field)
end

-- Schemas runner
function runSchemas(schemas)
  fieldsSum = v2(0,0)
  for i=1,#schemas do
  	field = fieldThreshold(schemas[i].schema(schemas[i].m), schemas[i].threshold * schemas[i].m)
  	if field.length > 0 and schemas[i].on ~= nil then
  		schemas[i].on(field)
  	elseif field.length == 0 and schemas[i].off ~= nil then
  		schemas[i].off(field)
  	end
  	fieldsSum = fieldsSum + field
  end
  return fieldsSum
end

-- Motors schemas
zero_field = v2P({angle = 0, length = 0})
function lightSchema(m)
	result = calcAvgLightingVector()
	result = ternary(result.length > 1, result.normalized, result)
	return result * m
end
function forwardSchema(m)
	return v2P({ angle = 0, length = 1}) * m
end
function obstacleSchema(m)
	result = calcMaxProximityVector()
	result = ternary(result.length > 1, result.normalized, result)
	return ternary(result.angle < math.pi*(3/4) and result.angle > -math.pi*(3/4), -result, zero_field) * m

end
function obstacleCircumSchema(m)
	result = -calcAvgProximityVector()
	result = result.rotate(-math.sign(result.angle) * (math.pi / 2) * 1.05).normalized
	return result.normalized * m
end

--debug callbacks
function avoidingObstacles(field) robot.leds.set_single_color(13, "red") end
function notAvoidingObstacles(field) robot.leds.set_single_color(13, "black") end
function followingLight(field) 
	for i=1,12 do robot.leds.set_single_color(i, "yellow") end 
end
function notFollowingLight(field) 
  for i=1,12 do robot.leds.set_single_color(i, "black") end 
 end

schemas = {
  { schema = forwardSchema,        m = 0.1,  threshold = 0.0 },
  { schema = obstacleSchema,       m = 2,  threshold = 0.1, on = avoidingObstacles, off = notAvoidingObstacles },
  { schema = obstacleCircumSchema, m = 4,  threshold = 0.2 },
  { schema = lightSchema,          m = 4,  threshold = 0.2, on = followingLight, off = notFollowingLight }
}
	
function step()

	
	
	
	fieldVector = runSchemas(schemas)
	vel = linearVelocity(fieldVector)
	
	m = math.max(math.abs(vel.x), math.abs(vel.y))
	vel = vel / m * MAX_VELOCITY

	
	robot.wheels.set_velocity(
	  math.max(-MAX_VELOCITY, math.min(MAX_VELOCITY, vel.x)), 
	  math.max(-MAX_VELOCITY, math.min(MAX_VELOCITY, vel.y)))
	
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
