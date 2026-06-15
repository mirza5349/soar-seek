import jsbsim
import os

fdm = jsbsim.FGFDMExec("/home/px4_sitl/sim_paper/jsbsim_src")
fdm.set_debug_level(1)
print("Loading model...")
fdm.load_model("soarer")
print("Loading IC...")
fdm.load_ic("init/cruise", True)
print("Running IC...")
fdm.run_ic()

print("Initial values:")
print("  qbar:", fdm['aero/qbar-psf'])
print("  cl-squared:", fdm['aero/cl-squared'])
print("  RPM:", fdm['propulsion/engine/propeller-rpm'])
print("  Thrust:", fdm['propulsion/engine/thrust-lbs'])

print("Setting engine running and throttle...")
fdm['propulsion/engine/set-running'] = 1
fdm['fcs/throttle-cmd-norm'] = 0.15 # Let's set a realistic low throttle first

print("Running simulation for 10 seconds to stabilize RPM...")
dt = fdm.get_delta_t()
for _ in range(int(10.0 / dt)):
    fdm.run()

print("Current RPM:", fdm['propulsion/engine/propeller-rpm'])
print("Current Thrust (lbs):", fdm['propulsion/engine/thrust-lbs'])
print("Attempting to trim...")
try:
    fdm['simulation/do_simple_trim'] = 0
    print("Trim Succeeded!")
    print("Trimmed Alpha:", fdm['aero/alpha-deg'])
    print("Trimmed Elevator:", fdm['fcs/elevator-pos-rad'] * 180.0 / 3.14159)
    print("Trimmed Throttle:", fdm['fcs/throttle-cmd-norm'])
    print("Trimmed Thrust (N):", fdm['propulsion/engine/thrust-lbs'] * 4.44822)
except Exception as e:
    print("Trim Failed:", e)



