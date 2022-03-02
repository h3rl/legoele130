function IntValueNew = Trapes(IntValueOld,FunctionValues,TimeStep)
    IntValueNew = IntValueOld + TimeStep * 0.5 * sum(FunctionValues);
end