function [] = ShortenMatfile(Filename, NewFilename, FromTime, ToTime)
    A = open(Filename);
    start_ = 0;
    slutt_ = 0;
    for n = 1: numel(A.Tid)
       tid = A.Tid(n);
       if tid <= FromTime
          start_ = n;
       end
       if tid <= ToTime
          slutt_ = n;
       end
    end
    
    fnames = fieldnames(A);
    for k=1:numel(fnames)
        fname = fnames{k};
        A.(fname) = A.(fname)(start_:slutt_);
    end
    
    A.Tid = A.Tid - A.Tid(1);
    
    save(NewFilename,"-struct","A");
end
