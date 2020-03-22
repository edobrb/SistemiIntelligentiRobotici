MAX_TOTAL_VELOCITY = 30  --cm/s
MAX_ROTATION_VELOCITY = 30 --cm/s
MAX_FORWARD_VELOCITY = 30 --cm/s
OBSTACLE_SENSIBILITY = 2
LIGHT_SENSIBILITY = 1
require('vector2')

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

function calcProximityVector()
	sum = v2(0,0)
	for i=1,#robot.proximity do
	    sensor_angle = -(-(math.pi * 2 / (#robot.proximity*2)) + i * math.pi * 2 / #robot.proximity)
		sum = sum + v2(math.cos(sensor_angle), math.sin(sensor_angle)) * robot.proximity[i].value
	end
	return -sum * OBSTACLE_SENSIBILITY
end
function calcLightingVector()
	sum = v2(0,0)
	for i=1,#robot.light do
	    sensor_angle = -(-(math.pi * 2 / (#robot.light * 2)) + i * math.pi * 2 / #robot.light)
		sum = sum + v2(math.cos(sensor_angle), math.sin(sensor_angle)) * robot.light[i].value
	end
	return sum * LIGHT_SENSIBILITY
end


function step()
	
	lights = calcLightingVector()
	prox = calcProximityVector()
	target_point = lights + prox
	
	proximity_factor = prox.length()
	lights_factor = lights.length()
	target_factor = target_point.length()
	
	target_angle = math.atan2(target_point.y, target_point.x)
	obstacle_angle = math.atan2(prox.y, prox.x)
	
	--Rotation direction depends on target_angle
	rotation_direction = math.sign(target_angle)
	
	--Rotation velocity depends on:
	--   1) target_angle (higher value means that the target is to the opposite direction so a faster rotation is preferred)
	--   2) target_factor (a high target_factor means a near wall or a near light. 
	--                                         If the target is very far the target_angle is calculated only 
	--                                         on noise levels and therefore a rotation is not needed =>  
	--                                         the robot is not perceiving a target: target_factor ~ 0)
	rotation_velocity = MAX_ROTATION_VELOCITY * math.abs(target_angle) * target_factor
	
	--Forward velocity depend on the presence of an obstacle in front of the robot, if the way is clear the robot go full speed
	forward_velocity = MAX_FORWARD_VELOCITY / (1 + math.abs(obstacle_angle) * proximity_factor) 
	
	left_v = math.max(math.min(forward_velocity + rotation_velocity * rotation_direction, MAX_TOTAL_VELOCITY), -MAX_TOTAL_VELOCITY)
	right_v = math.max(math.min(forward_velocity - rotation_velocity * rotation_direction, MAX_TOTAL_VELOCITY), -MAX_TOTAL_VELOCITY)
	robot.wheels.set_velocity(left_v, right_v)
	
end


function reset()
	robot.wheels.set_velocity(0,0)
end


function destroy()
   -- put your code here
end
