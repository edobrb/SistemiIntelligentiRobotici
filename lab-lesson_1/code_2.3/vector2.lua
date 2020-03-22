vector2_mt = {
    __add = function(left,right)
        return v2(left.x + right.x, left.y + right.y)
    end,
	__mul = function(left,right)
		return v2(left.x * right, left.y * right)
    end,
	__sub = function(left, right)
		return v2(left.x - right, left.y - right)
    end,
	__unm = function(left)
		return v2(-left.x , -left.y)
    end
}


function v2(X, Y)
	ret = { x = X, y = Y, 
	length = function() 
		return math.sqrt(X*X+Y*Y)
	end
	}
	setmetatable(ret, vector2_mt);
	return ret
end
