uint8 order         #order of calculus, (zero/first), or enum value for others if used
string term         #logical term, e.g. GOAL_REACHED in 0-order
bool value          #boolean value of term, e.g. GOAL_REACHED=true
float64 timeout      #UTC time, after which value shall be considered undefined
bool has_timeout    #if false, timeout shall be considered infinite
