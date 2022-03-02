function [IntValueNew] = EulerForward(IntValueOld,FunctionValue,TimeStep)
    IntValueNew = IntValueOld + FunctionValue * TimeStep;
end
