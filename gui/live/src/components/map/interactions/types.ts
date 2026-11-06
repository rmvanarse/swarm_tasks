import type { ModifyEvent } from 'ol/interaction/Modify';

import type { TransformFeaturesInteractionEvent } from './TransformFeatures';

export type BoxDragMode = 'add' | 'remove' | 'set';

export type FeatureUpdateType = 'modify' | 'transform';

export type TransformUpdateOptions = {
  event: TransformFeaturesInteractionEvent;
  type: 'transform';
};

export type ModifyUpdateOptions = {
  event: ModifyEvent;
  type: 'modify';
};

export type FeatureUpdateOptions = TransformUpdateOptions | ModifyUpdateOptions;

export type FeatureSelectionMode =
  'add' | 'remove' | 'clear' | 'toggle' | 'set';

export type FeatureSelectionOrActivationMode =
  FeatureSelectionMode | 'activate';
