/**
 * @file The search plan on the map: for every running search or split, the
 * area, each drone's strip of it (in the drone's own colour), the back-and-forth
 * lines it will sweep and the path it will really fly, with a label per drone.
 *
 * The plan comes from the swarm code through the server (GET /swarm/plan) and
 * is drawn here only; nothing on this layer can be edited.
 */

import createColor from 'color';
import { Fill, Stroke, Style, Text } from 'ol/style';
import { type ReactElement, useMemo } from 'react';
import { useSelector } from 'react-redux';

import { Feature, geom, layer, source } from '@collmot/ol-react';

import { getServerHttpUrl } from '~/features/servers/selectors';
import type { Layer } from '~/model/layers';
import { mapViewCoordinateFromLonLat } from '~/utils/geography';

import {
  colorOfBot,
  labelOfBot,
  type LatLon,
  type PlanBot,
  stripCentre,
  zigzag,
} from './model';
import { useSwarmPlan } from './useSwarmPlan';

export type SwarmPlanLayerParameters = {
  opacity: number;
  showStrips: boolean;
  showSearchLines: boolean;
  showPaths: boolean;
  showLabels: boolean;
};

type SwarmPlanLayerProps = {
  layer: Layer<Partial<SwarmPlanLayerParameters> | undefined>;
  zIndex?: number;
};

// === Helpers ===

/** OpenLayers coordinates of [lat, lon] pairs. */
const toMap = (points: LatLon[]) =>
  points.map(([lat, lon]) => mapViewCoordinateFromLonLat([lon, lat]));

const withAlpha = (color: string, alpha: number) =>
  createColor(color).alpha(alpha).rgb().string();

// Styles are the same for every strip of a colour and state: make each once.
const styleCache = new Map<string, Style>();

const cached = (key: string, create: () => Style): Style => {
  let style = styleCache.get(key);
  if (!style) {
    style = create();
    styleCache.set(key, style);
  }
  return style;
};

const outlineStyle = cached(
  'outline',
  () =>
    new Style({
      stroke: new Stroke({
        color: 'rgba(0, 0, 0, 0.85)',
        width: 2,
        lineDash: [10, 6],
      }),
    })
);

const stripStyle = (color: string, dimmed: boolean) =>
  cached(
    `strip-${color}-${dimmed}`,
    () =>
      new Style({
        fill: new Fill({ color: withAlpha(color, dimmed ? 0.07 : 0.2) }),
        stroke: new Stroke({
          color: withAlpha(color, dimmed ? 0.5 : 0.9),
          width: 1.5,
          lineDash: dimmed ? [4, 6] : undefined,
        }),
      })
  );

const searchLineStyle = (color: string, dimmed: boolean) =>
  cached(
    `lines-${color}-${dimmed}`,
    () =>
      new Style({
        stroke: new Stroke({
          color: withAlpha(color, dimmed ? 0.35 : 0.8),
          width: 1.5,
          lineDash: [6, 5],
        }),
      })
  );

const pathStyle = (color: string, dimmed: boolean) =>
  cached(
    `path-${color}-${dimmed}`,
    () =>
      new Style({
        stroke: new Stroke({
          color: withAlpha(color, dimmed ? 0.4 : 1),
          width: 2.5,
        }),
      })
  );

const labelStyle = (color: string, text: string) =>
  new Style({
    text: new Text({
      font: 'bold 13px sans-serif',
      text,
      textAlign: 'center',
      textBaseline: 'middle',
      fill: new Fill({ color: 'black' }),
      backgroundFill: new Fill({ color: 'rgba(255, 255, 255, 0.85)' }),
      backgroundStroke: new Stroke({ color, width: 2 }),
      padding: [2, 4, 2, 4],
    }),
  });

// === The layer ===

export const SwarmPlanLayer = ({
  layer: layerData,
  zIndex,
}: SwarmPlanLayerProps) => {
  const serverUrl = useSelector(getServerHttpUrl);
  const plan = useSwarmPlan(serverUrl);

  const {
    opacity = 0.9,
    showStrips = true,
    showSearchLines = true,
    showPaths = true,
    showLabels = true,
  } = layerData.parameters ?? {};

  const features = useMemo(() => {
    const out: ReactElement[] = [];

    for (const job of plan.jobs) {
      for (const [index, area] of job.areas.entries()) {
        if (area.ring.length >= 4) {
          out.push(
            <Feature key={`${job.id}-area-${index}`} style={outlineStyle}>
              <geom.Polygon coordinates={toMap(area.ring)} />
            </Feature>
          );
        }
      }

      for (const bot of job.bots) {
        out.push(...featuresOfBot(job.id, bot));
      }
    }

    function featuresOfBot(jobId: number, bot: PlanBot): ReactElement[] {
      const key = `${jobId}-bot-${bot.bot}`;
      const color = colorOfBot(bot.bot);
      const dimmed = bot.status !== 'active';
      const items: ReactElement[] = [];
      const { south, north, west, east } = bot.strip;

      if (showStrips) {
        const ring: LatLon[] = [
          [south, west],
          [south, east],
          [north, east],
          [north, west],
          [south, west],
        ];
        items.push(
          <Feature key={`${key}-strip`} style={stripStyle(color, dimmed)}>
            <geom.Polygon coordinates={toMap(ring)} />
          </Feature>
        );
      }

      if (showSearchLines && bot.sweeps.length > 0) {
        items.push(
          <Feature key={`${key}-lines`} style={searchLineStyle(color, dimmed)}>
            <geom.LineString coordinates={toMap(zigzag(bot.sweeps))} />
          </Feature>
        );
      }

      if (showPaths && bot.path.length > 1) {
        items.push(
          <Feature key={`${key}-path`} style={pathStyle(color, dimmed)}>
            <geom.LineString coordinates={toMap(bot.path)} />
          </Feature>
        );
      }

      if (showLabels) {
        items.push(
          <Feature
            key={`${key}-label`}
            style={labelStyle(color, labelOfBot(bot))}
          >
            <geom.Point coordinates={toMap([stripCentre(bot.strip)])[0]} />
          </Feature>
        );
      }

      return items;
    }

    return out;
  }, [plan, showStrips, showSearchLines, showPaths, showLabels]);

  return (
    <layer.Vector
      opacity={opacity}
      updateWhileAnimating
      updateWhileInteracting
      zIndex={zIndex}
    >
      <source.Vector>{features}</source.Vector>
    </layer.Vector>
  );
};
