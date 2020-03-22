v2_mt = {
  __add = function(left,right) return v2(left.x + right.x, left.y + right.y) end,
  __mul = function(left,right) return v2(left.x * right, left.y * right) end,
  __sub = function(left, right) return v2(left.x - right.x, left.y - right.y) end,
  __unm = function(left) return v2(-left.x , -left.y) end,
  __index = function(v, index) --in order to make v2 immutable
    if index == "length" then return math.sqrt(v.x ^ 2 + v.y ^ 2)
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
