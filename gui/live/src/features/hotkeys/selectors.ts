import { type AppSelector } from '~/store/reducers';

import { HotkeyScope } from './types';

export const getActiveHotkeyScope: AppSelector<HotkeyScope> = () =>
  HotkeyScope.GLOBAL;

export const getPendingUAVId: AppSelector<string> = (state) =>
  state.hotkeys.pendingUAVId;

export const isHotkeyDialogVisible: AppSelector<boolean> = (state) =>
  state.hotkeys.dialogVisible;

export const isPendingUAVIdOverlayVisible: AppSelector<boolean> = (state) =>
  state.hotkeys.pendingUAVId.length > 0;
