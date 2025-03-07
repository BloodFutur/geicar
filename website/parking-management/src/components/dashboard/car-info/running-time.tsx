import * as React from 'react';
import Avatar from '@mui/material/Avatar';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import Stack from '@mui/material/Stack';
import type { SxProps } from '@mui/material/styles';
import Typography from '@mui/material/Typography';
import { ClockClockwise as ClockClockwiseIcon} from '@phosphor-icons/react/dist/ssr/ClockClockwise';

export interface RunningTimeProps {
  sx?: SxProps;
  value: number;
}

export function RunningTime({ value, sx }: RunningTimeProps): React.JSX.Element {
  return (
    <Card sx={sx}>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'flex-start', justifyContent: 'space-between' }} spacing={3}>
          <Stack spacing={1}>
            <Typography color="text.secondary" variant="overline">
              Running Time
            </Typography>
            <Typography variant="h4">{value.toFixed(0)} mn</Typography>
          </Stack>
          <Avatar sx={{ backgroundColor: 'var(--mui-palette-secondary-main)', height: '56px', width: '56px' }}>
            <ClockClockwiseIcon fontSize="var(--icon-fontSize-lg)" />
          </Avatar> 
        </Stack>
      </CardContent>
    </Card>
  );
}
