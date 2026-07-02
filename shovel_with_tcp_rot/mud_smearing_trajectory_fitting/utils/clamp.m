function value = clamp(value, lowerBound, upperBound)
%CLAMP Clamp numeric values to the closed interval [lowerBound, upperBound].

value = min(max(value, lowerBound), upperBound);
end
