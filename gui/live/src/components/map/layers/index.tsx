import type React from 'react';

import { BackgroundHint } from '@skybrush/mui-components';

import { LayerType } from '~/model/layers';
import { BaseLayer, BaseLayerSettings } from './base';
import { GraticuleLayer } from './graticule';
import type { LayerProps } from './MapLayers';
import { SwarmPlanLayer, SwarmPlanLayerSettings } from './swarm-plan';
import { TerrainLayer, TerrainLayerSettings } from './terrain';
import { TileServerLayer, TileServerLayerSettings } from './tile-server';
export { MapLayers, type LayerConfig } from './MapLayers';

export type { LayerProps };

export const UnavailableLayerSettings = () => (
  <div key='_hint' style={{ position: 'relative', height: 48 }}>
    <BackgroundHint
      text='This layer is unavailable under the license of the
            currently connected server.'
    />
  </div>
);

export const layerSettingsComponents = {
  [LayerType.BASE]: BaseLayerSettings,
  [LayerType.SWARM_PLAN]: SwarmPlanLayerSettings,
  [LayerType.TERRAIN]: TerrainLayerSettings,
  [LayerType.TILE_SERVER]: TileServerLayerSettings,
  [LayerType.UNAVAILABLE]: UnavailableLayerSettings,
};

export const layerComponents: Partial<
  Record<LayerType, React.ComponentType<LayerProps>>
> = {
  [LayerType.BASE]: BaseLayer as React.ComponentType<LayerProps>,
  [LayerType.GRATICULE]: GraticuleLayer,
  [LayerType.SWARM_PLAN]: SwarmPlanLayer as React.ComponentType<LayerProps>,
  [LayerType.TERRAIN]: TerrainLayer as React.ComponentType<LayerProps>,
  [LayerType.TILE_SERVER]: TileServerLayer,
};
