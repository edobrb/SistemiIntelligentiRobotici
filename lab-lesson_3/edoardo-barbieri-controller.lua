require('vector2')

MAX_VELOCITY = 10 
AXIS_L = 0
zero_field = v2P({angle = 0, length = 0})

--[[ This function is executed every time you press the 'execute' button ]]
function init()
	robot.wheels.set_velocity(0,0)
	AXIS_L = robot.wheels.axis_length
end

-- (Transational velocity, Angular velocity) => Differential velocity (left, right)
function differentialVelocity(v)
	return v2(1, 1) * v.length + v2(-1, 1) * v.angle * AXIS_L / 2
end

-- SCHEMAS EXECUTOR
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
function fieldThreshold(v, threshold)
	return ternary(v.length > threshold, v, zero_field)
end


-- MOTOR SCHEMAS (every schemas must return a force field with maximum length m)
function forwardSchema(m)
	return v2P({ angle = 0, length = 1}) * m
end
function obstacleSchema(m)
	result = calcMaxProximityVector()
	result = ternary(result.length > 1, result.normalized, result)
	return -result * m / math.abs(result.angle)

end
function obstacleCircumSchema(m)
	result = -calcAvgProximityVector()
	result = result.rotate(-math.sign(result.angle) * (math.pi / 2) * 1.01) -- (almost) tangent field force
	result = ternary(result.length > 1, result.normalized, result)
	return result * m
end
function lightSchema(m)
	result = calcAvgLightingVector()
	result = ternary(result.length > 1, result.normalized, result)
	return result * m
end

--DEBUG CALLBACKS
function avoidingObstacles(field) robot.leds.set_single_color(13, "red") end
function notAvoidingObstacles(field) robot.leds.set_single_color(13, "black") end
function followingLight(field) for i=1,12 do if i%2 == 0 then robot.leds.set_single_color(i, "yellow") end end end
function notFollowingLight(field) for i=1,12 do if i%2 == 0 then robot.leds.set_single_color(i, "black") end end end
function followingWall(field) for i=1,12 do if i%2 == 1 then robot.leds.set_single_color(i, "green") end end end
function notFollowingWall(field) for i=1,12 do if i%2 == 1 then robot.leds.set_single_color(i, "black") end end end

-- SCHEMAS: every schema mest have: a 'schema' function that compute the force field, a multiplier 'm', a threshold 'threshold'
-- if the length of 'field' * 'm' is not > 'threshold' then the force field will be 0 for the specified schema.
-- When a schema surpass the threshold then 'on' is called, otherwise 'off'.
schemas = {
  { schema = forwardSchema,        m = 0.1, threshold = 0.0 },
  { schema = obstacleSchema,       m = 3,   threshold = 0.1, on = avoidingObstacles, off = notAvoidingObstacles },
  { schema = obstacleCircumSchema, m = 3,   threshold = 0.1, on = followingWall, off = notFollowingWall},
  { schema = lightSchema,          m = 4,   threshold = 0.2, on = followingLight, off = notFollowingLight }
}
	
function step()
	fieldVector = runSchemas(schemas)       --run the schemas and find compute the actual force field
	vel = differentialVelocity(fieldVector) -- convert it to differential velocity
	
	--max out the wheel velocity or keep inside MAX_VELOCITY
	m = math.max(math.abs(vel.x), math.abs(vel.y)) 
	vel = vel / m * MAX_VELOCITY

	robot.wheels.set_velocity(vel.x, vel.y)
end


function reset()
	robot.wheels.set_velocity(0,0)
	proximityMaxVector = v2(0,0)
	proximityAvgVector = v2(0,0)
	lightNearVector = v2(0,0)
	lightVector = v2(0,0)
end


function destroy()
   
end



-- UTILS
function ternary(c,t,f) if c then return t else return f end end
function math.sign(x) return ternary(x < 0, -1, ternary(x > 0, 1, 0)) end

-- READS FROM SENSORS UTILS
proximityMaxVector = v2(0,0)
proximityAvgVector = v2(0,0)
lightNearVector = v2(0,0)
lightVector = v2(0,0)
proximityMaxAlpha = 0.5
proximityAvgAlpha = 0.5
lightNearAlpha = 0.5
lightAlpha = 0.5
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
function calcAvgLightingVector()
	sum = v2(0,0)
	for i=1,#robot.light do
	    sensor_angle = robot.light[i].angle
		sum = sum + v2P({angle = robot.light[i].angle, length = robot.light[i].value})
	end
	lightVector =  lightVector * (1 - lightAlpha) + sum * lightAlpha
	return lightVector
end
--END SENSOR UTILS