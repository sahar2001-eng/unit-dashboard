# Turn machinery, vendored from the v2 upstream drop (2026-09-11).
#
#   rot_scan.py  scanning along a rotated direction (Algorithm 2, stage 2)
#   turn_sm.py   the gradient-orientation angle and the DRIVE/TURN state
#                machine (Algorithm 2, stage 1 + the machine)
#
# The upstream HostLink class is deliberately absent — see
# reference/pipeline_sync.json "restructured". Transport is the app's own
# CRC-checked link; TurnSM takes a send_event() sink instead.
