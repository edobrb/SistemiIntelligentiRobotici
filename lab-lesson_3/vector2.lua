v2_mt = {
  __add = function(left,right) return v2(left.x + right.x, left.y + right.y) end,
  __mul = function(left,right) return v2(left.x * right, left.y * right) end,
  __div = function(left,right) return v2(left.x / right, left.y / right) end,
  __sub = function(left, right) return v2(left.x - right.x, left.y - right.y) end,
  __unm = function(left) return v2(-left.x , -left.y) end,
  __index = function(v, index)
    if index == "length" then return math.sqrt(v.x ^ 2 + v.y ^ 2)
	elseif index == "angle" then return math.atan2(v.y, v.x)
	elseif index == "normalized" then if v.length==0 then return v2(0,0) else return v2(v.x / v.length, v.y / v.length) end
	elseif index == "rotate" then return function(angle) return v2P({length = v.length, angle = v.angle + angle}) end
    elseif index == "tostring" then return function() return "[" .. v.x ..", " .. v.y .. "]" end
    else return rawget(v, index) end end
}
function v2(X, Y)
  local v = { 
    x = X, 
    y = Y,
  }
  setmetatable(v, v2_mt)
  return v
end
function v2P(v) 
	return v2(v.length * math.cos(v.angle) , v.length * math.sin(v.angle))
end
