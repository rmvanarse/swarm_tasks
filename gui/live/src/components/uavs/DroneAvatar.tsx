import { createSelector } from '@reduxjs/toolkit';
import { connect } from 'react-redux';

import { ComplexAvatar, type ComplexAvatarProps } from '~/components/avatar';
import { getReverseMissionMapping } from '~/features/mission/selectors';
import { getBatteryFormatter } from '~/features/settings/selectors';
import {
  createSingleUAVStatusSummarySelector,
  getUAVById,
} from '~/features/uavs/selectors';
import {
  debugStringToAirspeed,
  parseSwarmMode,
} from '~/model/swarm-vehicle-type';
import type { RootState } from '~/store/reducers';
import { formatMissionId } from '~/utils/formatting';

type DroneAvatarVariant = 'full' | 'minimal';

type DroneAvatarStateProps = {
  batteryFormatter: ComplexAvatarProps['batteryFormatter'];
  hint?: string;
} & Partial<
  Pick<
    ComplexAvatarProps,
    | 'airspeed'
    | 'batteryStatus'
    | 'details'
    | 'gone'
    | 'mode'
    | 'status'
    | 'text'
    | 'textSemantics'
  >
>;

type DroneAvatarOwnProps = {
  id?: string;
  variant?: DroneAvatarVariant;
} & ComplexAvatarProps;

/**
 * Connected component that takes a ComplexAvatar and dresses it up to show the
 * status of a single drone.
 */
const DroneAvatar = connect(
  // mapStateToProps
  () => {
    const statusSummarySelector = createSingleUAVStatusSummarySelector();

    return createSelector(
      getBatteryFormatter,
      getReverseMissionMapping,
      (state: RootState, { id }: DroneAvatarOwnProps) =>
        statusSummarySelector(state, id ?? ''),
      (state: RootState, { id }: DroneAvatarOwnProps) =>
        id === undefined
          ? undefined
          : debugStringToAirspeed(getUAVById(state, id)?.debugString),
      (state: RootState, { id }: DroneAvatarOwnProps) =>
        id === undefined
          ? undefined
          : parseSwarmMode(getUAVById(state, id)?.mode).activity,
      (_state: RootState, ownProps: DroneAvatarOwnProps) => ownProps,
      (
        batteryFormatter,
        reverseMissionMapping,
        statusSummary,
        airspeed,
        mode,
        { hint, id, label, variant = 'full' }
      ): DroneAvatarStateProps => {
        const props: DroneAvatarStateProps = {
          batteryFormatter,
          airspeed,
          mode,
          ...statusSummary,
        };

        if (
          !hint &&
          (!label || label === id) &&
          id !== undefined &&
          id in reverseMissionMapping
        ) {
          props.hint = formatMissionId(reverseMissionMapping[id]);
        }

        if (variant !== 'full') {
          delete props.batteryStatus;
          delete props.text;
          delete props.details;
        }

        return props;
      }
    );
  },
  // mapDispatchToProps
  undefined,
  // mergeProps
  (
    stateProps: DroneAvatarStateProps,
    _dispatchProps,
    ownProps: DroneAvatarOwnProps
  ) => {
    const { variant: _variant, ...restOwnProps } = ownProps;
    return { ...restOwnProps, ...stateProps };
  }
)(ComplexAvatar);

export default DroneAvatar;
