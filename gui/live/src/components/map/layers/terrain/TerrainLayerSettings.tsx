/**
 * @file Settings of the terrain overlay: how it is drawn, how transparent it is,
 * and a button to download the elevation data around the current map view so
 * the overlay (and the terrain warnings) keep working without internet.
 */

import Button from '@mui/material/Button';
import Checkbox from '@mui/material/Checkbox';
import FormControl from '@mui/material/FormControl';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormLabel from '@mui/material/FormLabel';
import Radio from '@mui/material/Radio';
import RadioGroup from '@mui/material/RadioGroup';
import Slider from '@mui/material/Slider';
import TextField from '@mui/material/TextField';
import Typography from '@mui/material/Typography';
import { useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';

import { setLayerParametersById } from '~/features/map/layers';
import {
  getServerHttpUrl,
  supportsTerrain,
} from '~/features/servers/selectors';
import { showError, showNotification } from '~/features/snackbar/actions';
import { getMapViewCenterPosition } from '~/selectors/map';

import type { BaseLayerSettingsProps } from '../types';
import type { TerrainLayerParameters } from './TerrainLayer';

type PrefetchResult = {
  tiles: string[];
  result: Record<string, string[]>;
};

export const TerrainLayerSettings = ({
  layer,
  layerId,
}: BaseLayerSettingsProps<Partial<TerrainLayerParameters> | undefined>) => {
  const dispatch = useDispatch();
  const serverUrl = useSelector(getServerHttpUrl);
  const supported = useSelector(supportsTerrain);
  const [lon, lat] = useSelector(getMapViewCenterPosition);

  const [radiusKm, setRadiusKm] = useState('20');
  const [busy, setBusy] = useState(false);

  const {
    style = 'hillshade',
    opacity = 0.7,
    contours = true,
  } = layer.parameters ?? {};

  if (!supported) {
    return (
      <Typography color='error' variant='body2' sx={{ p: 1 }}>
        The connected server has no terrain extension. Enable{' '}
        <code>terrain</code> in the server configuration.
      </Typography>
    );
  }

  const download = async () => {
    const radius = Number(radiusKm);
    if (!Number.isFinite(radius) || radius <= 0) {
      showError('Enter a radius in kilometres.');
      return;
    }

    setBusy(true);
    try {
      const response = await fetch(`${serverUrl}/terrain/prefetch`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ lat, lon, radius_km: radius }),
      });
      if (!response.ok) {
        throw new Error(`the server answered ${response.status}`);
      }

      const { result }: PrefetchResult = await response.json();
      const count = (key: string) => result[key]?.length ?? 0;
      const ready = count('downloaded') + count('present');
      const problems = count('failed') + count('offline');
      const parts = [`${ready} terrain tile(s) ready`];
      if (count('downloaded')) parts.push(`${count('downloaded')} downloaded`);
      if (count('missing')) parts.push(`${count('missing')} have no data (sea)`);
      if (problems) {
        showError(
          `${parts.join(', ')}; ${problems} could not be downloaded ` +
            '(is the server online?)'
        );
      } else {
        showNotification(parts.join(', '));
      }
    } catch (error) {
      showError(
        `Could not download terrain data: ${
          error instanceof Error ? error.message : String(error)
        }`
      );
    } finally {
      setBusy(false);
    }
  };

  return (
    <div style={{ padding: 8 }}>
      <FormControl>
        <FormLabel>Style</FormLabel>
        <RadioGroup
          value={style}
          onChange={(_event, value) =>
            dispatch(setLayerParametersById(layerId, { style: value }))
          }
        >
          <FormControlLabel
            control={<Radio />}
            label='Shaded relief'
            value='hillshade'
          />
          <FormControlLabel
            control={<Radio />}
            label='Relief with height colours'
            value='relief'
          />
        </RadioGroup>
      </FormControl>

      <FormControlLabel
        control={
          <Checkbox
            checked={contours}
            onChange={(_event, checked) =>
              dispatch(setLayerParametersById(layerId, { contours: checked }))
            }
          />
        }
        label='Contour lines (show height on flat ground)'
      />

      <Typography gutterBottom sx={{ mt: 2 }} variant='body2'>
        Opacity
      </Typography>
      <Slider
        max={1}
        min={0.1}
        step={0.05}
        value={opacity}
        valueLabelDisplay='auto'
        valueLabelFormat={(value) => `${Math.round(value * 100)}%`}
        onChange={(_event, value) =>
          dispatch(setLayerParametersById(layerId, { opacity: value }))
        }
      />

      <Typography sx={{ mt: 2 }} variant='body2'>
        For use without internet, download the elevation data around the
        centre of the map while the server is online:
      </Typography>
      <div style={{ display: 'flex', gap: 8, alignItems: 'center', marginTop: 8 }}>
        <TextField
          label='Radius (km)'
          size='small'
          value={radiusKm}
          onChange={(event) => setRadiusKm(event.target.value)}
        />
        <Button
          disabled={busy}
          variant='outlined'
          onClick={() => {
            void download();
          }}
        >
          {busy ? 'Downloading…' : 'Download this area'}
        </Button>
      </div>
    </div>
  );
};
