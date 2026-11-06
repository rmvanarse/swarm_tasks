import { DialogContent } from '@mui/material';
import PropTypes from 'prop-types';
import { useCallback } from 'react';
import { useTranslation } from 'react-i18next';
import { connect } from 'react-redux';

import { DraggableDialog } from '@skybrush/mui-components';

import { getServerHttpUrl } from '~/features/servers/selectors';

import MapCachingPanel from './MapCachingPanel';
import { closeMapCachingDialog } from './slice';

const noPaddingStyle = { padding: 0 };

/**
 * Presentation component for the dialog that allows the user to adjust the
 * settings related to map caching on the server.
 */
const MapCachingDialog = ({ onClose, open, serverHttpUrl }) => {
  const { t } = useTranslation();

  const onClearCache = useCallback(async () => {
    try {
      await fetch(`${serverHttpUrl}/map-cache/clear`, { method: 'POST' });
    } catch (error) {
      console.warn('Could not clear the map cache on the server', error);
    }
  }, [serverHttpUrl]);

  return (
    <DraggableDialog
      fullWidth
      open={open}
      maxWidth='xs'
      title={t('mapCachingDialog.offlineMaps')}
      onClose={onClose}
    >
      <DialogContent sx={noPaddingStyle}>
        <MapCachingPanel onClearCache={onClearCache} onClose={onClose} />
      </DialogContent>
    </DraggableDialog>
  );
};

MapCachingDialog.propTypes = {
  open: PropTypes.bool,
  onClose: PropTypes.func,
  serverHttpUrl: PropTypes.string,
};

export default connect(
  // mapStateToProps
  (state) => ({
    open: state.mapCaching.dialog.open,
    serverHttpUrl: getServerHttpUrl(state),
  }),

  // mapDispatchToProps
  {
    onClose: closeMapCachingDialog,
  }
)(MapCachingDialog);
