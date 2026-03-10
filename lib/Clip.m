function y = Clip(x, lowerValue, upperValue)
    %#codegen
    
    y=min(max(x,lowerValue),upperValue);
end
