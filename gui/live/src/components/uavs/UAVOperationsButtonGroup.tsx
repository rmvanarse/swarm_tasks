import Assignment from '@mui/icons-material/Assignment';
import Clear from '@mui/icons-material/Clear';
import Delete from '@mui/icons-material/Delete';
import Explore from '@mui/icons-material/Explore';
import PositionHold from '@mui/icons-material/Flag';
import FlightLand from '@mui/icons-material/FlightLand';
import FlightTakeoff from '@mui/icons-material/FlightTakeoff';
import Home from '@mui/icons-material/Home';
import NearMe from '@mui/icons-material/NearMe';
import PersonAddAlt from '@mui/icons-material/PersonAddAlt';
import PersonRemove from '@mui/icons-material/PersonRemove';
import PlayArrow from '@mui/icons-material/PlayArrow';
import Stop from '@mui/icons-material/Stop';
import IconButton from '@mui/material/IconButton';
import { bindActionCreators } from '@reduxjs/toolkit';
import type { TransportOptions } from '@skybrush/flockwave-spec';
import isEmpty from 'lodash-es/isEmpty';
import { useTranslation } from 'react-i18next';
import { connect } from 'react-redux';

import Colors from '~/components/colors';
import ToolbarDivider from '~/components/ToolbarDivider';
import { TooltipWithContainerFromContext as Tooltip } from '~/containerContext';
import { getPreferredCommunicationChannelIndex } from '~/features/mission/selectors';
import {
  type AggregatedTaskState,
  createAggregatedTaskStateSelector,
  createTaskOperationThunks,
  TASK_SPECS,
} from '~/features/tasks';
import { sendAddCommand, sendRemoveCommand, sendStopCommand } from '~/features/swarm-ops/actions';
import {
  requestRemovalOfUAVsByIds,
  requestRemovalOfUAVsMarkedAsGone,
} from '~/features/uavs/actions';
import { UAV_TOOLBAR_MULTI_SELECTION_LIMIT } from '~/features/uavs/constants';
import { openUAVDetailsDialog } from '~/features/uavs/details';
import { getSelectedUAVIds, getUAVIdList } from '~/features/uavs/selectors';
import type { AppDispatch, RootState } from '~/store/reducers';
import { createUAVOperationThunks } from '~/utils/messaging';

type OwnProps = {
  broadcast: boolean;
  hideSeparators?: boolean;
  selectedUAVIds: string[];
  size?: 'small' | 'medium';
  startSeparator?: boolean;
};

type Props = OwnProps & {
  compassCalibrationStatus: AggregatedTaskState;
  dispatch: AppDispatch;
  getTargetedUAVIds: (state: RootState) => string[];
  openUAVDetailsDialog: (id: string) => void;
  requestRemovalOfUAVsByIds: (ids: string[]) => void;
  requestRemovalOfUAVsMarkedAsGone: () => void;
};

/**
 * Main toolbar for controlling the UAVs.
 */
const UAVOperationsButtonGroup = ({
  broadcast,
  compassCalibrationStatus,
  dispatch,
  getTargetedUAVIds,
  hideSeparators,
  openUAVDetailsDialog,
  requestRemovalOfUAVsByIds,
  requestRemovalOfUAVsMarkedAsGone,
  selectedUAVIds,
  size,
  startSeparator,
}: Props) => {
  const { t } = useTranslation();
  const isSelectionEmpty = isEmpty(selectedUAVIds) && !broadcast;
  const isSelectionSingle = selectedUAVIds.length === 1 && !broadcast;

  const {
    guidedMode,
    holdPosition,
    land,
    returnToHome,
    takeOff,
    turnMotorsOff,
    turnMotorsOn,
  } = bindActionCreators(
    createUAVOperationThunks({
      getTargetedUAVIds,
      getTransportOptions(state: RootState) {
        const result: TransportOptions = {
          channel: getPreferredCommunicationChannelIndex(state),
        };

        if (broadcast) {
          result.broadcast = true;
          result.ignoreIds = true;
        }

        return result;
      },
    }),
    dispatch
  );
  const { calibrateCompass } = bindActionCreators(
    createTaskOperationThunks({ getTargetedUAVIds: getSelectedUAVIds }),
    dispatch
  );

  const fontSize = size === 'small' ? 'small' : 'medium';
  const iconSize = size;
  const allowOperationForMultipleSelection =
    selectedUAVIds.length >= 1 &&
    selectedUAVIds.length <= UAV_TOOLBAR_MULTI_SELECTION_LIMIT;

  return (
    <>
      {!hideSeparators && startSeparator && (
        <ToolbarDivider orientation='vertical' />
      )}

      <Tooltip content={t('general.commands.takeoff')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={takeOff}
        >
          <FlightTakeoff fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      <Tooltip content={t('general.commands.positionHold')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={holdPosition}
        >
          <PositionHold fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      <Tooltip content={t('general.commands.guidedMode')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={guidedMode}
        >
          <NearMe fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      <Tooltip content={t('general.commands.returnToHome')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={returnToHome}
        >
          <Home fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      <Tooltip content={t('general.commands.land')}>
        <IconButton disabled={isSelectionEmpty} size={iconSize} onClick={land}>
          <FlightLand fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      {!hideSeparators && <ToolbarDivider orientation='vertical' />}

      {size !== 'small' && (
        <>
          <Tooltip content={t('UAVOpButtonGrp.details')}>
            <IconButton
              disabled={!isSelectionSingle}
              size={iconSize}
              onClick={() => openUAVDetailsDialog(selectedUAVIds[0])}
            >
              <Assignment />
            </IconButton>
          </Tooltip>
          <Tooltip content={t('general.commands.calibrateCompass')}>
            <IconButton
              size={iconSize}
              disabled={!allowOperationForMultipleSelection}
              onClick={() => calibrateCompass()}
              loading={compassCalibrationStatus.loading}
              color={
                compassCalibrationStatus.numItems === 0
                  ? undefined // No UAVs targeted
                  : compassCalibrationStatus.numError > 0
                    ? 'error' // At least one task failed
                    : compassCalibrationStatus.numSuccess ===
                        compassCalibrationStatus.numItems
                      ? 'success' // All tasks succeeded
                      : compassCalibrationStatus.numSuccess === 0
                        ? undefined // No tasks were run
                        : 'info' // Partial result
              }
            >
              <Explore />
            </IconButton>
          </Tooltip>
        </>
      )}

      {!hideSeparators && <ToolbarDivider orientation='vertical' />}

      <Tooltip content={t('UAVOpButtonGrp.armMotors')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={turnMotorsOn}
        >
          <PlayArrow
            fontSize={fontSize}
            htmlColor={isSelectionEmpty ? undefined : Colors.warning}
          />
        </IconButton>
      </Tooltip>

      <Tooltip content={t('UAVOpButtonGrp.disarmMotors')}>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={turnMotorsOff}
        >
          <Clear
            fontSize={fontSize}
            htmlColor={isSelectionEmpty ? undefined : Colors.warning}
          />
        </IconButton>
      </Tooltip>

      {size !== 'small' && (
        <>
          {!hideSeparators && <ToolbarDivider orientation='vertical' />}

          <Tooltip
            content={
              isSelectionEmpty
                ? t('UAVOpButtonGrp.removeItemsMarkedGone')
                : t('UAVOpButtonGrp.removeSelectedItems')
            }
          >
            <IconButton
              size={iconSize}
              onClick={() =>
                isSelectionEmpty
                  ? requestRemovalOfUAVsMarkedAsGone()
                  : requestRemovalOfUAVsByIds(selectedUAVIds)
              }
            >
              <Delete />
            </IconButton>
          </Tooltip>
        </>
      )}

      {!hideSeparators && <ToolbarDivider orientation='vertical' />}

      <Tooltip content='Stop (global -- stops whatever the swarm is currently running)'>
        <IconButton size={iconSize} onClick={() => void sendStopCommand()}>
          <Stop fontSize={fontSize} htmlColor={Colors.error} />
        </IconButton>
      </Tooltip>

      <Tooltip content='Remove selected bot(s) from the swarm (self-heal)'>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={() => void sendRemoveCommand(selectedUAVIds)}
        >
          <PersonRemove fontSize={fontSize} />
        </IconButton>
      </Tooltip>

      <Tooltip content='Add selected bot(s) back to the swarm'>
        <IconButton
          disabled={isSelectionEmpty}
          size={iconSize}
          onClick={() => void sendAddCommand(selectedUAVIds)}
        >
          <PersonAddAlt fontSize={fontSize} />
        </IconButton>
      </Tooltip>
    </>
  );
};

export default connect(
  // mapStateToProps factory
  () => {
    let current: OwnProps;
    const getTargetedUAVIds = (state: RootState): string[] =>
      current.broadcast ? getUAVIdList(state) : current.selectedUAVIds;

    const getCompassCalibrationStatus = createAggregatedTaskStateSelector(
      getTargetedUAVIds,
      TASK_SPECS.compassCalibration
    );

    return (state: RootState, ownProps: OwnProps) => {
      current = ownProps;
      return {
        getTargetedUAVIds,
        compassCalibrationStatus: getCompassCalibrationStatus(state),
      };
    };
  },
  // mapDispatchToProps
  (dispatch) => ({
    ...bindActionCreators(
      {
        openUAVDetailsDialog,
        requestRemovalOfUAVsMarkedAsGone,
        requestRemovalOfUAVsByIds,
      },
      dispatch
    ),
    dispatch,
  })
)(UAVOperationsButtonGroup);
