function [Derivative] = Derivation(FunctionValues,TimeStep)
    Derivative = (FunctionValues(1) - FunctionValues(2)) / TimeStep;
end