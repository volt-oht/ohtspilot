"""Experimental moving-lead response; does not command actuators.

The 0.3 m/s^2 extra allowance is a tuning candidate, not a validated vehicle limit.
Revision 005 extends eligibility to an already-authorized low-speed departure.
Vehicle benefit and low-speed response remain unvalidated.
"""
import math

ENABLED = True


def catchup_accel_limit(base_accel, enabled, active, v_ego, a_ego, t_follow, stop_distance, leads):
  """Return a bounded planning ceiling, failing to the original ceiling.

  Both leads must permit catch-up; an unavailable first lead cannot enable it.
  No distance reduction or change to braking, jerk costs or actuator commands.
  """
  if not enabled or not active or not 0.0 < base_accel <= 1.4:
    return base_accel
  values = (base_accel, v_ego, a_ego, t_follow, stop_distance)
  if not all(math.isfinite(x) for x in values):
    return base_accel
  if not (0.0 <= v_ego <= 80.0 / 3.6 and a_ego >= 0.0 and
          1.1 <= t_follow <= 2.7 and stop_distance >= 4.0):
    return base_accel
  if not leads or not leads[0].status:
    return base_accel
  strength = 1.0
  for lead in leads:
    if not lead.status:
      continue
    if not lead.radar:
      return base_accel
    if not all(math.isfinite(x) for x in (lead.dRel, lead.vRel, lead.vLead, lead.aLeadK)):
      return base_accel
    # A slowing or closing lead must retain the original ceiling immediately.
    relative_speed = min(lead.vRel, lead.vLead - v_ego)
    surplus = lead.dRel - (stop_distance + t_follow * v_ego)
    if lead.aLeadK < 0.0 or relative_speed <= 0.5 or surplus <= 2.0:
      return base_accel
    strength = min(strength, min((relative_speed - 0.5) / 1.5, 1.0),
                   min((surplus - 2.0) / 6.0, 1.0))
  # Preserve the prior ceiling above 2 m/s. Below it, taper the extra
  # headroom continuously to zero at 2 m/s; do not force a launch command.
  low_speed_headroom = 0.3 * max(0.0, 1.0 - v_ego / 2.0)
  ceiling = max(1.4, base_accel + low_speed_headroom)
  return min(ceiling, base_accel + 0.3 * strength)
