/**
 * @file Settings of the search plan layer: which parts of the plan to draw.
 */

import Checkbox from '@mui/material/Checkbox';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormGroup from '@mui/material/FormGroup';
import Slider from '@mui/material/Slider';
import Typography from '@mui/material/Typography';
import { useDispatch } from 'react-redux';

import { setLayerParametersById } from '~/features/map/layers';

import type { BaseLayerSettingsProps } from '../types';
import type { SwarmPlanLayerParameters } from './SwarmPlanLayer';

const PARTS: [keyof SwarmPlanLayerParameters, string][] = [
  ['showStrips', "Each drone's strip of the area"],
  ['showSearchLines', 'Search lines (the back-and-forth sweeps)'],
  ['showPaths', 'Flight path of each drone'],
  ['showLabels', 'Drone labels'],
];

export const SwarmPlanLayerSettings = ({
  layer,
  layerId,
}: BaseLayerSettingsProps<Partial<SwarmPlanLayerParameters> | undefined>) => {
  const dispatch = useDispatch();
  const parameters = layer.parameters ?? {};
  const { opacity = 0.9 } = parameters;

  return (
    <div style={{ padding: 8 }}>
      <Typography variant='body2' sx={{ mb: 1 }}>
        Shows what each drone is going to search while a search or split is
        planned or running. It is cleared when you press Stop.
      </Typography>

      <FormGroup>
        {PARTS.map(([name, label]) => (
          <FormControlLabel
            key={name}
            label={label}
            control={
              <Checkbox
                checked={parameters[name] !== false}
                onChange={(_event, checked) =>
                  dispatch(setLayerParametersById(layerId, { [name]: checked }))
                }
              />
            }
          />
        ))}
      </FormGroup>

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
    </div>
  );
};
