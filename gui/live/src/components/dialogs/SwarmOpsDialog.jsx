/**
 * @file Dialog exposing main.py's swarm-wide commands (search / split /
 * specific split / different heights / multi-waypoint goal) that don't fit
 * the stock "select UAVs, click a button" vocabulary. Stop and the per-UAV
 * add/remove self-heal commands are plain buttons elsewhere (see
 * UAVOperationsButtonGroup), not part of this dialog.
 *
 * Geographic inputs (waypoint paths, search center, split/specificsplit
 * sub-area centers) are never typed in by hand -- the operator draws them on the map
 * with the existing Draw path / Draw point tools (same ones
 * used for geofences etc.) and then picks the drawn feature from a dropdown
 * here. Map features store points as [lon, lat]; main.py's wire protocol
 * wants [lat, lon] -- toLatLonPairs() below is the one place that swap
 * happens.
 */

import Box from '@mui/material/Box';
import Button from '@mui/material/Button';
import Checkbox from '@mui/material/Checkbox';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import FormControlLabel from '@mui/material/FormControlLabel';
import FormGroup from '@mui/material/FormGroup';
import MenuItem from '@mui/material/MenuItem';
import Tab from '@mui/material/Tab';
import Tabs from '@mui/material/Tabs';
import Typography from '@mui/material/Typography';
import PropTypes from 'prop-types';
import { Field, Form } from 'react-final-form';
import { connect } from 'react-redux';

import { DraggableDialog } from '@skybrush/mui-components';

import { Select, TextField } from '~/components/forms';
import DronePlaceholderList from '~/components/uavs/DronePlaceholderList';
import { getFeaturesInOrder } from '~/features/map-features/selectors';
import {
  sendGoalCommand,
  sendSplitCommand,
  sendSearchCommand,
  sendSpecificSplitCommand,
  sendDifferentHeightsCommand,
} from '~/features/swarm-ops/actions';
import {
  closeSwarmOpsDialog,
  setSwarmOpsTab,
} from '~/features/swarm-ops/slice';
import { getSelectedUAVIds } from '~/features/uavs/selectors';
import { FeatureType } from '~/model/features';

/** Map feature points are [lon, lat] (GeoJSON order); main.py's wire
 * protocol wants [lat, lon] -- this is the single place that gets swapped.
 */
const toLatLonPairs = (points) => points.map(([lon, lat]) => [lat, lon]);

const featuresOfType = (features, type) =>
  features.filter((feature) => feature.type === type);

/** Dropdown of already-drawn map features of one geometry type, or an
 * explanatory message (styled as an error, since the form can't be
 * submitted without one) telling the operator which map tool to use.
 */
const FeatureField = ({ features, label, drawToolHint, name = 'featureId' }) =>
  features.length > 0 ? (
    <Select name={name} label={label} fullWidth margin='normal'>
      {features.map((feature) => (
        <MenuItem key={feature.id} value={feature.id}>
          {feature.label || feature.id} ({feature.points.length}{' '}
          {feature.points.length === 1 ? 'point' : 'points'})
        </MenuItem>
      ))}
    </Select>
  ) : (
    <Typography variant='body2' color='error' sx={{ mb: 1 }}>
      Nothing drawn yet. Switch to the <strong>{drawToolHint}</strong> map
      tool, draw it, then reopen this tab.
    </Typography>
  );

FeatureField.propTypes = {
  features: PropTypes.array,
  label: PropTypes.string,
  drawToolHint: PropTypes.string,
  name: PropTypes.string,
};

/** Checklist of drawn map points: each ticked point becomes one center
 * (sub-area) of a split. Needs at least one drawn point.
 */
const PointsChecklist = ({ features, drawToolHint, name = 'featureIds' }) =>
  features.length > 0 ? (
    <FormGroup sx={{ mb: 1 }}>
      {features.map((feature) => (
        <Field key={feature.id} name={name} type='checkbox' value={feature.id}>
          {({ input }) => (
            <FormControlLabel
              control={<Checkbox {...input} />}
              label={feature.label || feature.id}
            />
          )}
        </Field>
      ))}
    </FormGroup>
  ) : (
    <Typography variant='body2' color='error' sx={{ mb: 1 }}>
      Nothing drawn yet. Switch to the <strong>{drawToolHint}</strong> map
      tool, place the points, then reopen this tab.
    </Typography>
  );

PointsChecklist.propTypes = {
  features: PropTypes.array,
  drawToolHint: PropTypes.string,
  name: PropTypes.string,
};

/** [lat, lon] pairs for every point of the ticked features, in drawn order. */
const centersOf = (features, ids) =>
  toLatLonPairs(
    features
      .filter((feature) => (ids || []).includes(feature.id))
      .flatMap((feature) => feature.points)
  );

const GoalForm = ({ uavIds, features, onSubmit }) => {
  const pathFeatures = featuresOfType(features, FeatureType.LINE_STRING);
  return (
    <Form
      initialValues={{
        featureId: pathFeatures[0]?.id ?? '',
        direction: '1',
        radius: '50',
      }}
      onSubmit={(values) => {
        const feature = pathFeatures.find((f) => f.id === values.featureId);
        if (!feature) {
          return;
        }

        onSubmit(uavIds, {
          goal_latlon: toLatLonPairs(feature.points),
          direction: Number(values.direction),
          radius: Number(values.radius),
        });
      }}
    >
      {({ handleSubmit, values }) => (
        <Box>
          <DronePlaceholderList title='Selection:' items={uavIds} mt={0} mb={1} />
          <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
            Draw the route with the <strong>Draw path</strong> map tool, then
            pick it below.
          </Typography>
          <FeatureField
            features={pathFeatures}
            label='Waypoint path'
            drawToolHint='Draw path'
          />
          <TextField
            name='direction'
            label='Guided circle direction'
            fullWidth
            margin='normal'
          />
          <TextField
            name='radius'
            label='Guided circle radius (m)'
            fullWidth
            margin='normal'
          />
          <DialogActions>
            <Button
              color='primary'
              onClick={handleSubmit}
              disabled={uavIds.length === 0 || !values.featureId}
            >
              Send goal
            </Button>
          </DialogActions>
        </Box>
      )}
    </Form>
  );
};

GoalForm.propTypes = {
  uavIds: PropTypes.arrayOf(PropTypes.string),
  features: PropTypes.array,
  onSubmit: PropTypes.func,
};

const SearchForm = ({ features, onSubmit }) => {
  const pointFeatures = featuresOfType(features, FeatureType.POINTS);
  return (
    <Form
      initialValues={{
        featureId: pointFeatures[0]?.id ?? '',
        gridSpace: '10',
        coverageArea: '200',
      }}
      onSubmit={(values) => {
        const feature = pointFeatures.find((f) => f.id === values.featureId);
        if (!feature) {
          return;
        }

        const [lat, lon] = toLatLonPairs(feature.points)[0];
        onSubmit({
          center_lat: lat,
          center_lon: lon,
          grid_space: Number(values.gridSpace),
          coverage_area: Number(values.coverageArea),
        });
      }}
    >
      {({ handleSubmit, values }) => (
        <Box>
          <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
            Targets every active bot in the swarm -- UAV selection is
            ignored. Mark the center with the <strong>Draw point</strong> map
            tool, then pick it below.
          </Typography>
          <FeatureField
            features={pointFeatures}
            label='Center'
            drawToolHint='Draw point'
          />
          <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
          <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
          <DialogActions>
            <Button color='primary' onClick={handleSubmit} disabled={!values.featureId}>
              Start search
            </Button>
          </DialogActions>
        </Box>
      )}
    </Form>
  );
};

SearchForm.propTypes = { features: PropTypes.array, onSubmit: PropTypes.func };

const SplitForm = ({ uavIds, features, onSubmit }) => {
  const pointFeatures = featuresOfType(features, FeatureType.POINTS);
  return (
    <Form
      initialValues={{
        featureIds: pointFeatures.map((f) => f.id),
        gridSpace: '10',
        coverageArea: '200',
      }}
      onSubmit={(values) => {
        const centers = centersOf(pointFeatures, values.featureIds);
        if (centers.length === 0) {
          return;
        }

        onSubmit(uavIds, {
          center_lat_lon_array: centers,
          grid_space: Number(values.gridSpace),
          coverage_area: Number(values.coverageArea),
        });
      }}
    >
      {({ handleSubmit, values }) => (
        <Box>
          <DronePlaceholderList title='Selection:' items={uavIds} mt={0} mb={1} />
          <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
            Divides the work among the selected bots, one sub-area around each
            point. Place the points with the <strong>Draw point</strong> map
            tool, then tick the ones to use below.
          </Typography>
          <PointsChecklist
            features={pointFeatures}
            drawToolHint='Draw point'
          />
          <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
          <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
          <DialogActions>
            <Button
              color='primary'
              onClick={handleSubmit}
              disabled={uavIds.length === 0 || !(values.featureIds || []).length}
            >
              Split
            </Button>
          </DialogActions>
        </Box>
      )}
    </Form>
  );
};

SplitForm.propTypes = {
  uavIds: PropTypes.arrayOf(PropTypes.string),
  features: PropTypes.array,
  onSubmit: PropTypes.func,
};

const SpecificSplitForm = ({ features, onSubmit }) => {
  const pointFeatures = featuresOfType(features, FeatureType.POINTS);
  return (
    <Form
      initialValues={{
        featureIds: pointFeatures.map((f) => f.id),
        groups: '',
        gridSpace: '10',
        coverageArea: '200',
      }}
      onSubmit={(values) => {
        const centers = centersOf(pointFeatures, values.featureIds);
        if (centers.length === 0) {
          return;
        }

        onSubmit({
          center_lat_lon_array: centers,
          uav_array: values.groups
            .split(';')
            .map((group) =>
              group
                .split(',')
                .map((id) => Number(id.trim()))
                .filter((n) => !Number.isNaN(n))
            ),
          grid_space: Number(values.gridSpace),
          coverage_area: Number(values.coverageArea),
        });
      }}
    >
      {({ handleSubmit, values }) => (
        <Box>
          <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
            Same points as Split, but you hand-pick which bots cover which
            point. Place the points with the <strong>Draw point</strong> map
            tool, then tick the ones to use below (in drawn order). Groups:
            bot ids per point, separated by ";" (e.g. "1,2;3,4" = bots 1+2 to
            the first point, 3+4 to the second).
          </Typography>
          <PointsChecklist
            features={pointFeatures}
            drawToolHint='Draw point'
          />
          <TextField name='groups' label='Bot id groups' placeholder='1,2;3,4' fullWidth margin='normal' />
          <TextField name='gridSpace' label='Grid spacing (m)' fullWidth margin='normal' />
          <TextField name='coverageArea' label='Coverage area (m)' fullWidth margin='normal' />
          <DialogActions>
            <Button color='primary' onClick={handleSubmit} disabled={!(values.featureIds || []).length}>
              Specific split
            </Button>
          </DialogActions>
        </Box>
      )}
    </Form>
  );
};

SpecificSplitForm.propTypes = { features: PropTypes.array, onSubmit: PropTypes.func };

const DifferentHeightsForm = ({ onSubmit }) => (
  <Form
    initialValues={{ start: '100', diff: '10' }}
    onSubmit={(values) =>
      onSubmit({ start: Number(values.start), diff: Number(values.diff) })
    }
  >
    {({ handleSubmit }) => (
      <Box>
        <Typography variant='body2' color='textSecondary' sx={{ mb: 1 }}>
          Sets every bot's commanded altitude at once -- selection is
          ignored.
        </Typography>
        <TextField name='start' label='Start height (m)' fullWidth margin='normal' />
        <TextField name='diff' label='Height step between bots (m)' fullWidth margin='normal' />
        <DialogActions>
          <Button color='primary' onClick={handleSubmit}>
            Apply heights
          </Button>
        </DialogActions>
      </Box>
    )}
  </Form>
);

DifferentHeightsForm.propTypes = { onSubmit: PropTypes.func };

const TABS = [
  { value: 'goal', label: 'Goal' },
  { value: 'search', label: 'Search' },
  { value: 'split', label: 'Split' },
  { value: 'specificsplit', label: 'Specific split' },
  { value: 'different', label: 'Heights' },
];

/** Keeps the dialog out of the way: docked to the top right, with no backdrop
 * and pointer events passed through everywhere except the dialog itself, so
 * the UAV list and the map stay usable (e.g. to select UAVs or draw features)
 * while it is open. It can still be dragged elsewhere by its title bar.
 */
const nonModalSx = {
  pointerEvents: 'none',
  '& .MuiDialog-container': {
    alignItems: 'flex-start',
    justifyContent: 'flex-end',
  },
  '& .MuiDialog-paper': {
    pointerEvents: 'auto',
    marginTop: '64px',
    marginRight: '16px',
  },
};

const SwarmOpsDialog = ({
  open,
  selectedTab,
  uavIds,
  features,
  onClose,
  onTabChange,
}) => (
  <DraggableDialog
    hideBackdrop
    disableEnforceFocus
    disableScrollLock
    open={open}
    maxWidth='xs'
    title='Swarm ops'
    sx={nonModalSx}
    onClose={onClose}
  >
    <Tabs
      value={selectedTab}
      onChange={(_event, value) => onTabChange(value)}
      variant='scrollable'
    >
      {TABS.map((tab) => (
        <Tab key={tab.value} value={tab.value} label={tab.label} />
      ))}
    </Tabs>
    <DialogContent>
      {selectedTab === 'goal' && (
        <GoalForm uavIds={uavIds} features={features} onSubmit={sendGoalCommand} />
      )}
      {selectedTab === 'search' && (
        <SearchForm features={features} onSubmit={sendSearchCommand} />
      )}
      {selectedTab === 'split' && (
        <SplitForm uavIds={uavIds} features={features} onSubmit={sendSplitCommand} />
      )}
      {selectedTab === 'specificsplit' && (
        <SpecificSplitForm features={features} onSubmit={sendSpecificSplitCommand} />
      )}
      {selectedTab === 'different' && (
        <DifferentHeightsForm onSubmit={sendDifferentHeightsCommand} />
      )}
    </DialogContent>
  </DraggableDialog>
);

SwarmOpsDialog.propTypes = {
  open: PropTypes.bool,
  selectedTab: PropTypes.string,
  uavIds: PropTypes.arrayOf(PropTypes.string),
  features: PropTypes.array,
  onClose: PropTypes.func,
  onTabChange: PropTypes.func,
};

export default connect(
  // mapStateToProps
  (state) => ({
    open: state.swarmOps.dialog.open,
    selectedTab: state.swarmOps.dialog.selectedTab,
    uavIds: getSelectedUAVIds(state),
    features: getFeaturesInOrder(state),
  }),
  // mapDispatchToProps
  {
    onClose: closeSwarmOpsDialog,
    onTabChange: setSwarmOpsTab,
  }
)(SwarmOpsDialog);
