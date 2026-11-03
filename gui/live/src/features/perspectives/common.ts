import { type PerspectiveName, type PerspectiveObject } from 'perspective';

/* Object holding well-known panel layouts that may potentially be used in
 * multiple configuration files. Perspective configurations may refer to these
 * by their names only */
const commonLayouts: Record<PerspectiveName, PerspectiveObject> = {
  default: {
    label: 'Default',
    layout: {
      type: 'columns',
      contents: [
        {
          type: 'stack',
          contents: [
            { type: 'panel', component: 'map', id: 'map' },
            { type: 'panel', component: 'uav-list', id: 'uavList' },
          ],
        },
        {
          type: 'rows',
          contents: [
            {
              type: 'stack',
              contents: [
                {
                  type: 'panel',
                  component: 'saved-location-list',
                  id: 'locations',
                },
                { type: 'panel', component: 'layer-list', id: 'layers' },
              ],
              height: 25,
            },
            {
              type: 'stack',
              contents: [
                { type: 'panel', component: 'connection-list', id: 'connections' },
                { type: 'panel', component: 'messages', id: 'messages' },
              ],
            },
          ],
          width: 25,
        },
      ],
    },
  },
};

export default commonLayouts;
