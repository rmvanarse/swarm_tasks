import { createNextState } from '@reduxjs/toolkit';
import { createMigrate } from 'redux-persist';

import { BatteryThresholdType } from '~/features/safety/model';
import {
  DEFAULT_BATTERY_CELL_COUNT,
  LIPO_CRITICAL_VOLTAGE_THRESHOLD,
  LIPO_FULL_CHARGE_VOLTAGE,
  LIPO_LOW_VOLTAGE_THRESHOLD,
} from '~/model/constants';
import { getDefaultLabelForFeature } from '~/model/features';
import { chooseUniqueName } from '~/utils/naming';

const migrations = {
  2: createNextState((state) => {
    state.settings.uavs = {
      ...state.settings.uavs,
      defaultBatteryCellCount: DEFAULT_BATTERY_CELL_COUNT,
      fullChargeVoltage: LIPO_FULL_CHARGE_VOLTAGE,
      lowVoltageThreshold: LIPO_LOW_VOLTAGE_THRESHOLD,
      criticalVoltageThreshold: LIPO_CRITICAL_VOLTAGE_THRESHOLD,
    };
  }),
  3: createNextState((state) => {
    if (typeof state.safety.settings.lowBatteryVoltage === 'number') {
      state.safety.settings.lowBatteryThreshold = {
        type: BatteryThresholdType.VOLTAGE,
        value: state.safety.settings.lowBatteryVoltage,
      };
    }
  }),
  // Every map feature gets a label ("Point", "Point 1", "Path", ...), so
  // drawn points and paths show a name in the map and in the goal dialog.
  4: createNextState((state) => {
    const features = state.features;
    if (!features || !features.byId) {
      return;
    }

    const ids = features.order ?? Object.keys(features.byId);
    const used = ids
      .map((id) => features.byId[id]?.label)
      .filter((label) => typeof label === 'string' && label.length > 0);

    for (const id of ids) {
      const feature = features.byId[id];
      if (feature && !feature.label) {
        feature.label = chooseUniqueName(
          getDefaultLabelForFeature(feature.type, id),
          used
        );
        used.push(feature.label);
      }
    }
  }),
  // The "Search plan" layer (what each drone is going to search) is added to
  // layer lists that were saved before it existed, just above the features.
  5: createNextState((state) => {
    const layers = state.map && state.map.layers;
    if (!layers || !layers.byId || !layers.order) {
      return;
    }

    if (
      Object.values(layers.byId).some((layer) => layer.type === 'swarmPlan')
    ) {
      return;
    }

    let id = 'swarmPlan';
    while (layers.byId[id]) {
      id += '_';
    }

    layers.byId[id] = {
      id,
      type: 'swarmPlan',
      label: 'Search plan',
      visible: true,
      parameters: {
        opacity: 0.9,
        showStrips: true,
        showSearchLines: true,
        showPaths: true,
        showLabels: true,
      },
    };

    const at = layers.order.findIndex(
      (layerId) => layers.byId[layerId]?.type === 'features'
    );
    layers.order.splice(at >= 0 ? at + 1 : layers.order.length, 0, id);
  }),
};

export default createMigrate(migrations);
