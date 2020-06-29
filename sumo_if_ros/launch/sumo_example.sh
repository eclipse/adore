#!/bin/bash
cd ${SUMO_HOME}
./bin/sumo --remote-port 1337 -c tests/complex/tutorial/quickstart/data/quickstart.sumocfg --step-length 0.01
