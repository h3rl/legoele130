function [Derivative] = Derivation(FunctionValues,TimeStep)
    Derivative = (FunctionValues(2) - FunctionValues(1)) / TimeStep;
end