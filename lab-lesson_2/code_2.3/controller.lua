-- Considerazioni:
-- Il codice risulta piuttosto offuscato e non sono riuscito a mantenere 
-- un principio di semplicità nei task che sono andato a sviluppare (presenza di molte casistiche if else)
-- Non è presente un'architettura ben definita per il controllo a sussunzione.
-- Avendo più tempo a disposizione sicuramente avrei ripensato da zero la soluzione.
-- Non troppo chiari i vantaggi nell'implementare un sistema di controllo a sussunzione

-- Descrizione del comportamento voluto del robot:
-- 1. Il robot in assenza di luce e di ostacoli va dritto
-- 2. Il robot in presenza di sola luce va verso la luce e rallenta all'avvicinarsi di essa
-- 3. Il robot in presenza di un ostacolo cerca di circumnavigarlo tenendolo alla sua destra o sinistra
-- 4. il robot in presenza di più ostacoli cerca di allontanarsi da essi
-- 5. il robot in presenza di una luce dal lato opposto dell'ostacolo che sta circumnavigando
--      deve lasciare perdere l'ostacolo e andare verso la luce

-- Soluzione:

-- Ogni livello può avere come output i livelli di potenza delle due ruote. Un livello
-- superiore può andare a sovrascrivere questi valori.

-- Livelli (tasks) individuati (dal più base al più avanzato):
--   sensing_level: analizza i dati raccolti dai sensori, nessun output
--   forward_level: livello dedicato a far andar dritto il robot
--   obstacle_avoidance_level: questo livello aggiunge l'abilità di evitare gli ostacoli
--   keep_wall_to_right_level: questo livello aggiunge l'abilità nel circumnavigare un ostacolo
--   phototaxis_level: questo livello permette di far andare il robot verso la luce quando è più opportuno farlo


MAX_TOTAL_VELOCITY = 30  --cm/s
MAX_ROTATION_VELOCITY = 30 --cm/s
MAX_FORWARD_VELOCITY = 30 --cm/s
OBSTACLE_SENSIBILITY = 1
LIGHT_SENSIBILITY = 2
LIMIT_OBSTACLE_MAGNITUDE = 0.8

require('vector2')

function init()
	robot.wheels.set_velocity(0,0)
end

-- UTILS
function math.sign(x)
   if x<0 then
     return -1
   elseif x>0 then
     return 1
   else
     return 0
   end
end

function calcProximityVector()
	sum = v2(0,0)
	for i=1,#robot.proximity do
	    sensor_angle = robot.proximity[i].angle
		sum = sum + v2(math.cos(sensor_angle), math.sin(sensor_angle)) * robot.proximity[i].value
	end
	return sum * OBSTACLE_SENSIBILITY
end

function calcLightingVector()
	sum = v2(0,0)
	for i=1,#robot.light do
	    sensor_angle = robot.light[i].angle
		sum = sum + v2(math.cos(sensor_angle), math.sin(sensor_angle)) * robot.light[i].value
	end
	return sum * LIGHT_SENSIBILITY
end

-- SUBSUNTIVE LEVELS
function sensing_level(data)
	sensing_data = { }
	v = calcProximityVector()
	sensing_data["obstacle_is_present"] = v.length > 0.1
	sensing_data["obstacle_magnitude"] = v.length
	sensing_data["obstacle_direction"] = math.atan2(v.y, v.x)
	sensing_data["obstacle_position"] = v
	v1 = calcLightingVector()
	sensing_data["light_is_present"] = v1.length > 0.2
	sensing_data["light_magnitude"] = v1.length
	sensing_data["light_direction"] = math.atan2(v1.y, v1.x)
	sensing_data["light_position"] = v1
	data["sensing"] = sensing_data
end

function forward_level(data)
	data.left_motor_power = 1
	data.right_motor_power = 1
end

function obstacle_avoidance_level(data)
	if data.sensing.obstacle_is_present then
		alpha = 1 / data.sensing.obstacle_magnitude
		data.left_motor_power = alpha + math.sign(data.sensing.obstacle_direction) * (1-alpha)
		data.right_motor_power = alpha - math.sign(data.sensing.obstacle_direction)* (1-alpha)
		urgent = (math.pi - math.abs(data.sensing.obstacle_direction)) / math.pi * 2
		data.left_motor_power = 1 + (math.sign(data.sensing.obstacle_direction) * urgent)
		data.right_motor_power = 1 - (math.sign(data.sensing.obstacle_direction) * urgent)
	end
end

wall_present = false 	-- if the robot has found a wall and it's fallowing it wall_present is true
keep_left = true  -- wall fallowing direction
function keep_wall_to_right_level(data)
	
	-- understand where is the obstacle (right or left)
	left = false
	right = false
	for i=1,12 do
	    if robot.proximity[i].value > LIMIT_OBSTACLE_MAGNITUDE/3 then
			left = true
		end
	end
	for i=13,#robot.proximity do
	    if robot.proximity[i].value > LIMIT_OBSTACLE_MAGNITUDE/3 then
			right = true
		end
	end
	if right and left then -- let obstacle avoidance level do the work
		return
	end

	-- FALLOWING THE WALL LOGIC
	if data.sensing.obstacle_is_present and (not wall_present) then
		wall_present = true
		keep_left = data.sensing.obstacle_direction > 0
	end
	angle = (data.sensing.obstacle_direction)
	if keep_left then 
		angle = angle - math.pi/2
	else
		angle = angle + math.pi/2
	end
	
	if math.abs(angle) > math.pi/16 and data.sensing.obstacle_is_present then 
		data.left_motor_power = -math.sign(angle)
		data.right_motor_power = math.sign(angle)
	elseif wall_present then
		if keep_left then
			if data.sensing.obstacle_magnitude < LIMIT_OBSTACLE_MAGNITUDE then
				data.left_motor_power = 0.5
				data.right_motor_power = 1
			else
				data.left_motor_power = 1
				data.right_motor_power = 0.5
			end
		else
			if data.sensing.obstacle_magnitude < LIMIT_OBSTACLE_MAGNITUDE then
				data.left_motor_power = 1
				data.right_motor_power = 0.5
			else
				data.left_motor_power = 0.5
				data.right_motor_power = 1
			end
		end
	end
end

function phototaxis(data)
		data.left_motor_power = (1 - data.sensing.light_direction * data.sensing.light_magnitude) / data.sensing.light_magnitude
		data.right_motor_power = (1 + data.sensing.light_direction * data.sensing.light_magnitude) / data.sensing.light_magnitude
end


function phototaxis_level(data)
	angle = data.sensing.light_direction - data.sensing.obstacle_direction
	angle = math.abs(((angle + math.pi) % (2*math.pi)) - math.pi)
	if data.sensing.obstacle_magnitude < LIMIT_OBSTACLE_MAGNITUDE then						-- off wall
		if wall_present and angle > math.pi/2 and data.sensing.light_is_present then				-- light at the opposite direction of wall (and fallowing wall)
			phototaxis(data)
			wall_present = false
		elseif (not wall_present) and data.sensing.light_is_present then											-- not fallowing the wall and light is present
			phototaxis(data)
		end
	end

end

function step()
	
	data = { left_motor_power = 0.0 , right_motor_power = 0.0 }
	
	-- COMPUTING LEVELS
	sensing_level(data)
	forward_level(data)
	obstacle_avoidance_level(data)
	keep_wall_to_right_level(data)
	phototaxis_level(data)
	
	
	-- SETTING WHEEL VELOCITY 
	robot.wheels.set_velocity(0, 0)
	left_v = math.max(math.min(data.left_motor_power * MAX_TOTAL_VELOCITY, MAX_TOTAL_VELOCITY), -MAX_TOTAL_VELOCITY)
	right_v = math.max(math.min(data.right_motor_power * MAX_TOTAL_VELOCITY, MAX_TOTAL_VELOCITY), -MAX_TOTAL_VELOCITY)
	robot.wheels.set_velocity(left_v, right_v)
end


function reset()
	robot.wheels.set_velocity(0,0)
end


function destroy()
   -- put your code here
end
