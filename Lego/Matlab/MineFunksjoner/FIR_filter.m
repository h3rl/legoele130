function [ FilteredValue ] = FIR_filter(Measurements ,NoOfMeas)
    size = numel(Measurements);
    if size < NoOfMeas
        NoOfMeas = size;
    end
    FilteredValue = 1/NoOfMeas * sum(Measurements(size - NoOfMeas + 1:size));
end