#!/bin/bash
cd ${1}
./bin/sumo -c ${2} --remote-port ${3} --step-length ${4} 
bash