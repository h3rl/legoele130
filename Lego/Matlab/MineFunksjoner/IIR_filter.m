function [FilteredValue] = IIR_filter(OldFilteredValue,Measurement,Para)
    FilteredValue = Para*Measurement + (1-Para)*OldFilteredValue;
end