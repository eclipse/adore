#defines state of a connection as controlled by TCD
Connection connection # the referenced connection
float64 time # time when state becomes active (future time for state prediction)
int8 state # state, values as defined below
int8 UNKNOWN = 0 # no observation available
int8 WAIT = 1 # wait at connection start (i.e. traffic light red)
int8 STOP = 2 # stop at connection start then yield to traffic (i.e. traffic light flashing yellow + stop sign, or stop sign only)
int8 YIELD = 3 # yield to traffic, no stop required (i.e. traffic light flashing yellow + yield)
int8 GO = 4 # free to use connection (i.e. traffic light green); yield only to higher priority connections, which are not GO
