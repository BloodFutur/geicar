import * as React from 'react';
import Avatar from '@mui/material/Avatar';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import Stack from '@mui/material/Stack';
import type { SxProps } from '@mui/material/styles';
import Typography from '@mui/material/Typography';
import { CarBattery as CarBatteryIcon } from '@phosphor-icons/react/dist/ssr/CarBattery';
import LinearProgress from '@mui/material/LinearProgress';

export interface BatteryProps {
  sx?: SxProps;
  value: number;
}

export function Battery({ value, sx }: BatteryProps): React.JSX.Element {
  return (
    <Card sx={sx}>
      <CardContent>
        <Stack spacing={2}>
          <Stack direction="row" sx={{ alignItems: 'flex-start', justifyContent: 'space-between' }} spacing={3}>
            <Stack spacing={1}>
              <Typography color="text.secondary" variant="overline">
                Battery
              </Typography>
              <Typography variant="h4">{value.toFixed(1)}V</Typography>
            </Stack>
            <Avatar sx={{ backgroundColor: (value < 11.5) ? 'var(--mui-palette-error-main)' : ((value >= 12.5) ? 'var(--mui-palette-success-main)' : 'var(--mui-palette-warning-main)'), height: '56px', width: '56px' }}>
              <CarBatteryIcon fontSize="var(--icon-fontSize-lg)" />
            </Avatar>
          </Stack>
          <div>
            <LinearProgress value={(value - 11) / 0.03} variant="determinate" color={(value < 11.5) ? 'error' : ((value >= 12.5) ? 'success' : 'warning')} />
          </div>
        </Stack>
      </CardContent>
    </Card>
  );
}
