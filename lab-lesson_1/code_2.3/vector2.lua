v2_mt = {
  __add = function(left,right) return v2(left.x + right.x, left.y + right.y) end,
  __mul = function(left,right) return v2(left.x * right, left.y * right) end,
  __sub = function(left, right) return v2(left.x - right.x, left.y - right.y) end,
  __unm = function(left) return v2(-left.x , -left.y) end
}

function v2(X, Y)
	v = { 
    x = X, 
    y = Y, 
    length = function() return math.sqrt(X ^ 2 + Y ^ 2) end,
    tostring = function() return "[" .. X ..", " .. Y .. "]" end,
    type = "v2"
  }
  setmetatable(v, v2_mt);
  return v
end
