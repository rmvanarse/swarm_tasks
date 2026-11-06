import Hub from '@mui/icons-material/Hub';
import IconButton from '@mui/material/IconButton';
import { useDispatch } from 'react-redux';

import { TooltipWithContainerFromContext as Tooltip } from '~/containerContext';

import { openSwarmOpsDialog } from './slice';

/**
 * Toolbar button that opens the swarm ops dialog.
 */
const SwarmOpsButton = () => {
  const dispatch = useDispatch();

  return (
    <Tooltip content='Swarm ops (search / split / specific split / heights / multi-waypoint goal)'>
      <IconButton onClick={() => dispatch(openSwarmOpsDialog())}>
        <Hub />
      </IconButton>
    </Tooltip>
  );
};

export default SwarmOpsButton;
