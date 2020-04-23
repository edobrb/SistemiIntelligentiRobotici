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

MAXRANGE=30
W=0.1
S=0.15
PsMAX=0.99
PwMIN=0.01
ALPHA=0.1
BETA=0.05

MAX_VELOCITY = 10
AXIS_L = 0
currentStep = 0
MOVING_STATE = "m"
STANDING_STATE = "s"
currentState = MOVING_STATE

--[[ This function is executed every time you press the 'execute' button ]]
function init()
	robot.wheels.set_velocity(0,0)
	AXIS_L = robot.wheels.axis_length
end

-- (Transational velocity, Angular velocity) => Differential velocity (left, right)
function differentialVelocity(v)
	return (v2(1, 1) * v.length) + (v2(-1, 1) * v.angle) * (AXIS_L / 2)
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
	return ternary(v.length > threshold, v, v2(0,0))
end



-- MOTOR SCHEMAS (every schemas must return a force field with maximum length m)
randomAngle = 0
function randomWalkSchema(m)
    if currentStep % 5 == 0 then
		randomAngle = (robot.random.uniform() - 0.5) / 10
	end
	return v2P({ angle = randomAngle, length = 1}) * m
end
function obstacleSchema(m)
	result = calcMaxProximityVector()
	return -result * m
end


-- SCHEMAS: every schema mest have: a 'schema' function that compute the force field, a multiplier 'm', a threshold 'threshold'
-- if the length of 'field' * 'm' is not > 'threshold' then the force field will be 0 for the specified schema.
-- When a schema surpass the threshold then 'on' is called, otherwise 'off'.
schemas = {
  { schema = randomWalkSchema,   m = 1, threshold = 0.0 },
  { schema = obstacleSchema,           m = 2, threshold = 0.3 },
}

function step()
	currentStep = currentStep + 1
	fieldVector =  v2(0,0)
	robot.range_and_bearing.set_data(1, ternary(moving, 0, 1))
	N = countRAB()

    --IF BLACK SPOT ARE PRESENT (EXERCISE 2/3) --
	D = 0
	if robot.motor_ground ~= nil then
		sum = 0
		for i=1,#robot.motor_ground do
			sum = sum + robot.motor_ground[i].value
       end
	   D = ternary(sum == 0, 0.2, 0)
	end

	--PROBABILISITC AUTOMATON--
	if currentState == MOVING_STATE then
	  fieldVector =  runSchemas(schemas) -- walk
	  Ps = math.min(PsMAX, S + ALPHA * N + D)
	  p = robot.random.uniform()
	  if Ps > p then currentState = STANDING_STATE  end --stop
	
	elseif  currentState == STANDING_STATE then
	  Pw = math.max(PwMIN, W - BETA * N - D)
	  p = robot.random.uniform()
	  if Pw > p then currentState = MOVING_STATE  end --walk
	end
	
	
	
	--FIELD BASED MOVE--
	vel = differentialVelocity(fieldVector)
	m = math.max(math.abs(vel.x), math.abs(vel.y)) 
	if m == 0 then
		vel = v2(0,0)
	else
		vel = vel / m * MAX_VELOCITY
	end
	robot.wheels.set_velocity(vel.x, vel.y)
	
end


function reset()

end


function destroy()
   log(robot.positioning.position.x .. "|" .. robot.positioning.position.y)
end



-- UTILS
function ternary(c,t,f) if c then return t else return f end end
function math.sign(x) return ternary(x < 0, -1, ternary(x > 0, 1, 0)) end

-- READS FROM SENSORS UTILS
function countRAB()
  number_robot_sensed = 0
  for i = 1, #robot.range_and_bearing do
-- for each robot seen, check they it is close enough.
    if robot.range_and_bearing[i].range < MAXRANGE and
      robot.range_and_bearing[i].data[1]==1 then
      number_robot_sensed = number_robot_sensed + 1
    end
  end
  return number_robot_sensed
end


proximityMaxVector = v2(0,0)
proximityMaxAlpha = 1
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
--END SENSOR UTILS