/**
 * @file What the server tells Live about the search plan (GET /swarm/plan):
 * for every running search or split, the area, each drone's strip of it, the
 * search lines it will sweep and the path it will really fly. Positions are
 * [latitude, longitude] pairs, as sent by the swarm code.
 */

export type LatLon = [number, number];

/** "active" while the drone works on it; "left" when it was given other work
 * (its strip is still being finished by another drone); "removed" when it was
 * taken out of the swarm. */
export type PlanBotStatus = 'active' | 'left' | 'removed';

export type PlanBot = {
  /** The swarm's own number for the drone (position in the swarm) */
  bot: number;
  /** MAVLink system id, which is the UAV id Live shows */
  sys_id: number | null;
  /** Index into the area list of the job */
  area: number;
  strip: { south: number; north: number; west: number; east: number };
  /** One [start, end] per search line; all run west to east */
  sweeps: [LatLon, LatLon][];
  path: LatLon[];
  status: PlanBotStatus;
};

export type PlanArea = {
  /** Closed outline, [lat, lon] */
  ring: LatLon[];
  center: LatLon;
  size_m: number;
  /** Bots that work in this area */
  bots: number[];
};

export type PlanJob = {
  id: number;
  kind: string;
  label: string;
  areas: PlanArea[];
  bots: PlanBot[];
};

export type SwarmPlan = {
  /** Changes whenever the plan does; -1 while the swarm code is unreachable */
  version: number;
  jobs: PlanJob[];
  reachable?: boolean;
};

export const EMPTY_PLAN: SwarmPlan = { version: -2, jobs: [] };

/** A colour for each drone, so its strip, lines and label match. */
export const BOT_COLORS = [
  '#e6194b',
  '#3cb44b',
  '#4363d8',
  '#f58231',
  '#911eb4',
  '#0097a7',
  '#f032e6',
  '#7cb342',
  '#9a6324',
  '#5d4037',
] as const;

export const colorOfBot = (bot: number): string =>
  BOT_COLORS[(Math.max(1, bot) - 1) % BOT_COLORS.length];

/**
 * The search lines as the drone flies them: east, then back west on the next
 * line, and so on. Returns [lat, lon] points.
 */
export function zigzag(sweeps: [LatLon, LatLon][]): LatLon[] {
  const points: LatLon[] = [];
  sweeps.forEach(([start, end], index) => {
    if (index % 2 === 0) {
      points.push(start, end);
    } else {
      points.push(end, start);
    }
  });
  return points;
}

/** "UAV 08", and "(bot 3)" when the swarm's number for it is different. */
export function labelOfBot(bot: PlanBot): string {
  const sys = bot.sys_id ?? bot.bot;
  const name = `UAV ${String(sys).padStart(2, '0')}`;
  const extra = sys === bot.bot ? '' : ` (bot ${bot.bot})`;
  const state =
    bot.status === 'left'
      ? ' - left'
      : bot.status === 'removed'
        ? ' - removed'
        : '';
  return `${name}${extra}${state}`;
}

/** Middle of a strip, [lat, lon]. */
export function stripCentre(strip: PlanBot['strip']): LatLon {
  return [(strip.south + strip.north) / 2, (strip.west + strip.east) / 2];
}

/** Whether Live has anything to draw. */
export const hasPlan = (plan: SwarmPlan): boolean =>
  plan.jobs.some((job) => job.bots.length > 0);
