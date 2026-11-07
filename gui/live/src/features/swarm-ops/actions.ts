/**
 * @file Thunks that send this project's swarm commands to the server as one
 * `X-SWARM-CMD` message each. The server's swarm_bridge extension
 * (gui/skybrush-server/src/flockwave/server/ext/swarm_bridge/) turns the
 * command into the text format main.py expects and forwards it over UDP.
 *
 * These are fleet-level operations, so they are not routed through any UAV
 * driver: drone telemetry and stock vehicle controls come from the server's
 * mavlink extension, swarm operations from this message.
 */

import { showError } from '~/features/snackbar/actions';
import { getUAVIdList } from '~/features/uavs/selectors';
import messageHub from '~/message-hub';
import store from '~/store';
import { runMassOperation, type Outcomes } from '~/utils/messaging';

/**
 * Fleet-wide commands (stop, search, specific split, heights) ignore the
 * selection; the ids are only used for the confirmation text and the
 * success/failure notification.
 */
const allKnownUAVIds = (): string[] => getUAVIdList(store.getState());

/**
 * Builds a sender for one named swarm command. `kwds` keys must match the
 * `_build_<command>()` parameters in swarm_bridge/driver.py exactly.
 */
function createSwarmCommandSender(
  command: string,
  name: string,
  { skipConfirmation = false }: { skipConfirmation?: boolean } = {}
) {
  return async (
    uavIds: string[],
    kwds: Record<string, unknown> = {}
  ): Promise<void> => {
    await runMassOperation(store.dispatch, () => store.getState(), {
      name,
      uavIds,
      isBroadcast: false,
      skipConfirmation,
      // sendSwarmMessage shows its own error toast with main.py's reason.
      reportFailure: false,
      run: async () => {
        const outcome = await sendSwarmMessage(command, name, uavIds, kwds);
        const outcomes: Outcomes = {};
        for (const id of uavIds) {
          outcomes[id] = outcome;
        }
        return outcomes;
      },
    });
  };
}

async function sendSwarmMessage(
  command: string,
  name: string,
  uavIds: string[],
  kwds: Record<string, unknown>
): Promise<'success' | 'failure'> {
  try {
    // The server answers only after main.py has accepted or rejected the
    // command, so a NAK here carries main.py's own reason.
    const response: any = await messageHub.sendMessage({
      type: 'X-SWARM-CMD',
      command,
      uavIds,
      kwds,
    });
    if (response?.body?.type === 'ACK-NAK') {
      showError(`${name} rejected: ${response.body.reason || 'unknown reason'}`);
      return 'failure';
    }

    return 'success';
  } catch (error) {
    console.error(`Swarm command ${command} failed`, error);
    const reason = error instanceof Error ? error.message : String(error);
    showError(`${name} rejected: ${reason}`);
    return 'failure';
  }
}

/** goal_<json goal_latlon>_<direction>_<radius>_<json ids> */
export const sendGoalCommand = createSwarmCommandSender('goal', 'Goal');

/** stop -- global stop of whatever main.py is currently running. */
const sendStop = createSwarmCommandSender('stop', 'Stop', {
  skipConfirmation: true,
});
export const sendStopCommand = () => sendStop(allKnownUAVIds(), {});

/** search,<center_lat>,<center_lon>,<num_uavs>,<grid_space>,<coverage_area> */
const sendSearch = createSwarmCommandSender('search', 'Search', {
  skipConfirmation: true,
});
export const sendSearchCommand = (kwds: Record<string, unknown>) =>
  sendSearch(allKnownUAVIds(), kwds);

/** split_<json center_lat_lon_array>_<json selected_uav_ids>_<json grid_space>_<json coverage_area> */
export const sendSplitCommand = createSwarmCommandSender('split', 'Split');

/** specificsplit_<json center_lat_lon_array>_<json uav_array>_<json grid_space>_<json coverage_area> */
const sendSpecificSplit = createSwarmCommandSender(
  'specificsplit',
  'Specific split',
  { skipConfirmation: true }
);
export const sendSpecificSplitCommand = (kwds: Record<string, unknown>) =>
  sendSpecificSplit(allKnownUAVIds(), kwds);

/** different,<start>,<diff> -- sets every bot's commanded height. */
const sendDifferent = createSwarmCommandSender(
  'different',
  'Different heights',
  { skipConfirmation: true }
);
export const sendDifferentHeightsCommand = (kwds: Record<string, unknown>) =>
  sendDifferent(allKnownUAVIds(), kwds);

/** remove_<json [id]> -- self-heal: freeze the given bots. */
export const sendRemoveCommand = createSwarmCommandSender(
  'remove',
  'Remove from swarm'
);

/** add_<json [id]> -- bring the given bots back into service. */
export const sendAddCommand = createSwarmCommandSender(
  'add',
  'Add back to swarm'
);
