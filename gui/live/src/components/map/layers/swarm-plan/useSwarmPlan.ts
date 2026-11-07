import { useEffect, useState } from 'react';

import { EMPTY_PLAN, type SwarmPlan } from './model';

/**
 * Asks the server for the search plan every couple of seconds while the
 * layer is shown, and returns the latest one. The state only changes when the
 * plan does (the server numbers every change), so the map is not redrawn for
 * nothing.
 */
export function useSwarmPlan(
  serverUrl: string | undefined,
  intervalMs = 2000
): SwarmPlan {
  const [plan, setPlan] = useState<SwarmPlan>(EMPTY_PLAN);

  useEffect(() => {
    if (!serverUrl) {
      setPlan(EMPTY_PLAN);
      return undefined;
    }

    let cancelled = false;
    let timer: number | undefined;

    const poll = async () => {
      try {
        const response = await fetch(`${serverUrl}/swarm/plan`, {
          cache: 'no-store',
        });
        if (response.ok) {
          const data = (await response.json()) as SwarmPlan;
          if (!cancelled && Array.isArray(data.jobs)) {
            setPlan((previous) =>
              previous.version === data.version &&
              previous.reachable === data.reachable
                ? previous
                : data
            );
          }
        }
      } catch {
        // The server does not have the swarm extension, or is not reachable
        // right now: keep showing what we have and try again.
      }

      if (!cancelled) {
        timer = window.setTimeout(() => void poll(), intervalMs);
      }
    };

    void poll();

    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [serverUrl, intervalMs]);

  return plan;
}
