# Variables for simulator
export INSTANCE=0
export LAT=42.3898
export LON=-71.1476
export ALT=14
export DIR=270
export MODEL=+
export SPEEDUP=1
export VEHICLE=ArduCopter

# Finally the command
 /home/robby/Mothership/ardupilot/Tools/autotest/sim_vehicle.py --vehicle ${VEHICLE} -I${INSTANCE} --custom-location=${LAT},${LON},${ALT},${DIR} -w --frame ${MODEL} --no-rebuild --no-mavproxy --speedup ${SPEEDUP}