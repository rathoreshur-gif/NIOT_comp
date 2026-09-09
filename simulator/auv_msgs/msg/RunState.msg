# The state of one scored competition run, published by auv_scoring/run_manager
# on /simulator/run_state at 5 Hz.
#
# This crosses the domain bridge into the competitor's domain, so it is how a
# controller learns that it is live and how much time it has left. Commands go
# back the other way as plain strings on /simulator/run_control: domain_bridge
# carries topics, not services.

uint8 IDLE=0        # armed but not started; thrusters are dead
uint8 RUNNING=1     # clock running, scoring live
uint8 FINISHED=2    # frozen; reset_run is the only way out

std_msgs/Header header      # stamp is SIM time

uint8 state
string state_label          # "IDLE" | "RUNNING" | "FINISHED", for humans and logs

float64 elapsed             # seconds since start_run; 0 before it, frozen after end_run
float64 time_limit          # seconds allowed, from run.time_limit in the config
float64 remaining           # max(0, time_limit - elapsed) - what the time bonus pays for

int32 score                 # live total from the score keeper, time bonus included once paid

# Whether each task has been run at all, on either side and for any points. The
# time bonus is only paid when both are true; see run.time_bonus in the config.
bool gate_complete
bool slalom_complete

uint32 run_index            # increments on every start_run and reset_run
string run_id               # course identity, e.g. "20260818_101112_seed1044476335"
string message              # why the run ended; empty while it has not
