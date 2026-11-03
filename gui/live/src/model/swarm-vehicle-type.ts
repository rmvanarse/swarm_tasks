/**
 * @file Parses the swarm_bridge extension's "mode" encoding
 * (`<vehicle_type>:<activity>`, e.g. "ground:goal" / "fixedwing:loiter")
 * back into separate fields for the UI.
 *
 * This is a stand-in for a dedicated `vehicleType` status field -- see the
 * "verify-then-upgrade" note in the project plan on why it isn't one yet
 * (the external flockwave-spec package that defines UAVStatusInfo's wire
 * schema isn't vendored in this repo, so adding a field to it needed to be
 * verified empirically before relying on it). If that upgrade lands, swap
 * this parser out for a direct read of the dedicated field -- every call
 * site in the UI goes through this one function.
 */

export type SwarmVehicleType = 'ground' | 'copter' | 'fixedwing';

/**
 * The server's mavlink driver announces the vehicle kind and airspeed in the
 * status "debug" field as "<plane|copter>[;as=<m/s>]".
 */
export const isVehicleTypeDebugString = (value: string | undefined): boolean =>
  /^(plane|copter)(;|$)/.test(value ?? '');

/** Airspeed in m/s from the announced debug string, if present. */
export const debugStringToAirspeed = (
  value: string | undefined
): number | undefined => {
  const match = /;as=(-?\d+(?:\.\d+)?)/.exec(value ?? '');
  return match ? Number(match[1]) : undefined;
};

export type ParsedSwarmMode = {
  vehicleType: SwarmVehicleType | undefined;
  activity: string;
};

const KNOWN_VEHICLE_TYPES: SwarmVehicleType[] = ['ground', 'fixedwing'];

/** Maps the server-announced debug string to a vehicle type. */
export const debugStringToVehicleType = (
  value: string | undefined
): SwarmVehicleType | undefined =>
  /^plane(;|$)/.test(value ?? '')
    ? 'fixedwing'
    : /^copter(;|$)/.test(value ?? '')
      ? 'copter'
      : undefined;

/**
 * Splits a swarm_bridge-encoded mode string into its vehicle type and
 * activity parts. Returns `vehicleType: undefined` for anything that
 * doesn't match the encoding (e.g. a real flight-mode string from a
 * different driver), so callers can fall back to showing `mode` as-is.
 */
export const parseSwarmMode = (mode: string | undefined): ParsedSwarmMode => {
  if (!mode) {
    return { vehicleType: undefined, activity: '' };
  }

  const [prefix, ...rest] = mode.split(':');
  if ((KNOWN_VEHICLE_TYPES as string[]).includes(prefix)) {
    return {
      vehicleType: prefix as SwarmVehicleType,
      activity: rest.join(':'),
    };
  }

  return { vehicleType: undefined, activity: mode };
};

/** Short label for the vehicle-type pill in the UAV list. */
export const vehicleTypeLabel = (
  vehicleType: SwarmVehicleType | undefined
): string => {
  switch (vehicleType) {
    case 'ground':
    case 'copter':
      return 'COPTR';
    case 'fixedwing':
      return 'PLANE';
    default:
      return '-----';
  }
};
