-- CONSIDERAZIONI:
-- Con l'architettura motor schemas mi sono trovato più a mio agio in quanto modellare il comportamento del robot
-- basandosi su campi di forza mi è sembrato più intuitivo, infatti le mie prime soluzioni avevano qualcosa in comune
-- con questa architettura. Tuttavia mi rendo conto che questa architettura da sola non potrà essere la soluzione 
-- per task complessi.
-- Credo che si potrebbero ottenere dei risultati migliori andando a unire i pregi di un'architettura 
-- motor schemas ad una tra sussunzione o behaviour trees. con la prima andrei ad implementare i comportamenti
-- base del robot, mentre con la seconda andrei ad orchestrare i comportamenti in base al contesto in cui
-- si trova attualemnte il robot.


require('vector2')

MAX_VELOCITY = 10 
AXIS_L = 0
zero_field = v2P({angle = 0, length = 0})
steps = 0
lightFoundStep = -1
collisions = 0

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
	result = result.rotate(-math.sign(result.angle) * (math.pi / 2) * robot.params.ALMOST_TANGENT) -- (almost) tangent field force
	result = ternary(result.length > 1, result.normalized, result)
	return result * m
end
function lightSchema(m)
	result = calcAvgLightingVector()
	result = ternary(result.length > 1, result.normalized, result)
	return result * m
end

-- SCHEMAS: every schema mest have: a 'schema' function that compute the force field, a multiplier 'm', a threshold 'threshold'
-- if the length of 'field' * 'm' is not > 'threshold' then the force field will be 0 for the specified schema.
-- When a schema surpass the threshold then 'on' is called, otherwise 'off'.
schemas = {
  { schema = forwardSchema,  m = robot.params.FORWARD_SCHEMA_M, threshold = 0.0 },
  { schema = obstacleSchema,  m = robot.params.OBSTACLE_SCHEMA_M,   threshold = robot.params.OBSTACLE_SCHEMA_T},
  { schema = obstacleCircumSchema, m = robot.params.OBSTACLE_CIRCUM_SCHEMA_M,   threshold =  robot.params.OBSTACLE_CIRCUM_SCHEMA_T},
  { schema = lightSchema,  m = robot.params.LIGHT_CIRCUM_SCHEMA_M,   threshold = robot.params.LIGHT_CIRCUM_SCHEMA_T },
  { schema = lightSchema,  m = robot.params.LIGHT_CIRCUM_SCHEMA_M2,   threshold = robot.params.LIGHT_CIRCUM_SCHEMA_T2 }
}
	
function step()
	steps = steps + 1
	fieldVector = runSchemas(schemas)       --run the schemas and find compute the actual force field
	vel = differentialVelocity(fieldVector) -- convert it to differential velocity
	
	--max out the wheel velocity or keep inside MAX_VELOCITY
	m = math.max(math.abs(vel.x), math.abs(vel.y)) 
	vel = vel / m * MAX_VELOCITY

	robot.wheels.set_velocity(vel.x, vel.y)
	
	if lightDistance() < 0.2 and lightFoundStep == -1 then
		lightFoundStep = steps
	end
	collided = false
	for i=2,#robot.proximity do
		if(robot.proximity[i].value > 0.99) then
			collided = true
		end
	end
	if collided then
		collisions = collisions + 1
	end
 
end


function reset()
	robot.wheels.set_velocity(0,0)
	proximityMaxVector = v2(0,0)
	proximityAvgVector = v2(0,0)
	lightNearVector = v2(0,0)
	lightVector = v2(0,0)
end

function lightDistance() 
	return (v2(robot.positioning.position.x, robot.positioning.position.y) - v2(-3, -3)).length
end
function destroy()
		cost = lightDistance() * 0.5 + ternary(lightFoundStep == -1, 1, lightFoundStep / steps)* 1 + (collisions / steps) * 30
		log("<cost>" .. cost .. "</cost>")
end



-- UTILS
function ternary(c,t,f) if c then return t else return f end end
function math.sign(x) return ternary(x < 0, -1, ternary(x > 0, 1, 0)) end

-- READS FROM SENSORS UTILS
proximityMaxVector = v2(0,0)
proximityAvgVector = v2(0,0)
lightVector = v2(0,0)
proximityMaxAlpha = robot.params.PROXIMITY_MAX_ALPHA
proximityAvgAlpha = robot.params.PROXIMITY_AVG_ALPHA
lightAlpha = robot.params.LIGHT_ALPHA
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