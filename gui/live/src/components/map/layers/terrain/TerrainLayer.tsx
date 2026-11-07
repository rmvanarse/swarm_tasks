/**
 * @file The terrain overlay: shaded relief (or height-tinted relief) tiles drawn
 * by the server's `terrain` extension from its elevation data. The tiles are
 * mostly transparent so the layer can sit on top of a satellite or street base
 * layer; its opacity is set in the layer's settings.
 */

import { layer as olLayer, source } from '@collmot/ol-react';
import { useSelector } from 'react-redux';

import {
  getServerHttpUrl,
  supportsTerrain,
} from '~/features/servers/selectors';
import type { Layer } from '~/model/layers';

export type TerrainStyle = 'hillshade' | 'relief';

export type TerrainLayerParameters = {
  style: TerrainStyle;
  opacity: number;
  /** Thin height lines on top of the shading; they show flat ground too. */
  contours: boolean;
};

type TerrainLayerProps = {
  layer: Layer<Partial<TerrainLayerParameters> | undefined>;
  zIndex?: number;
};

/** Highest zoom the server draws; above it the tiles are stretched. */
const MAX_TERRAIN_ZOOM = 17;

export const TerrainLayer = ({ layer, zIndex }: TerrainLayerProps) => {
  const serverUrl = useSelector(getServerHttpUrl);
  const supported = useSelector(supportsTerrain);

  // Without the server extension there is nothing to draw (and no point in
  // sending requests that would fail).
  if (!serverUrl || !supported) {
    return null;
  }

  const { style = 'hillshade', opacity = 0.7, contours = true } =
    layer.parameters ?? {};

  return (
    <olLayer.Tile opacity={opacity} zIndex={zIndex}>
      <source.XYZ
        attributions='Terrain: NASA SRTM'
        maxZoom={MAX_TERRAIN_ZOOM}
        url={`${serverUrl}/terrain/tiles/{z}/{x}/{y}.png?style=${style}&contours=${contours ? 1 : 0}`}
      />
    </olLayer.Tile>
  );
};
