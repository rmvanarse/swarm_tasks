/**
 * @file Slice of the state object that handles the "Swarm Ops" dialog --
 * the swarm-wide commands (search / split / specific split / different
 * heights / multi-waypoint goal) that main.py exposes but that don't fit
 * the stock "select UAVs, click a button" vocabulary.
 */

import { createSlice, type PayloadAction } from '@reduxjs/toolkit';

import { noPayload } from '~/utils/redux';

export type SwarmOpsTab =
  | 'goal'
  | 'search'
  | 'split'
  | 'specificsplit'
  | 'different';

type SwarmOpsSliceState = {
  dialog: {
    open: boolean;
    selectedTab: SwarmOpsTab;
  };
};

const initialState: SwarmOpsSliceState = {
  dialog: {
    open: false,
    selectedTab: 'goal',
  },
};

const { actions, reducer } = createSlice({
  name: 'swarm-ops',
  initialState,
  reducers: {
    openSwarmOpsDialog: noPayload<SwarmOpsSliceState>((state) => {
      state.dialog.open = true;
    }),

    closeSwarmOpsDialog: noPayload<SwarmOpsSliceState>((state) => {
      state.dialog.open = false;
    }),

    setSwarmOpsTab(state, { payload }: PayloadAction<SwarmOpsTab>) {
      state.dialog.selectedTab = payload;
    },
  },
});

export const { openSwarmOpsDialog, closeSwarmOpsDialog, setSwarmOpsTab } =
  actions;

export default reducer;
