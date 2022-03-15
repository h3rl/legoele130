function clampedvalue = Clamp(value, min_,max_)
  % return clamped value between min and max
  clampedvalue=min(max(value,min_),max_);