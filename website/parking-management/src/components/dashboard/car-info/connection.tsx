import * as React from 'react';
import Avatar from '@mui/material/Avatar';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import Stack from '@mui/material/Stack';
import type { SxProps } from '@mui/material/styles';
import Typography from '@mui/material/Typography';
import { CellTower as CellTowerIcon} from '@phosphor-icons/react/dist/ssr/CellTower';

export interface ConnectionProps {
  sx?: SxProps;
  value: boolean;
}

export function Connection({ value, sx }: ConnectionProps): React.JSX.Element {
  return (
    <Card sx={sx}>
      <CardContent>
        <Stack direction="row" sx={{ alignItems: 'flex-start', justifyContent: 'space-between' }} spacing={3}>
          <Stack spacing={1}>
            <Typography color="text.secondary" variant="overline">
              Connection
            </Typography>
            <Typography variant="h4">{value ? "On" : "Off" }</Typography>
          </Stack>
          <Avatar sx={{ backgroundColor: value ? 'var(--mui-palette-success-main)' : 'var(--mui-palette-error-main)', height: '56px', width: '56px' }}>
            <CellTowerIcon fontSize="var(--icon-fontSize-lg)" />
          </Avatar>
        </Stack>
      </CardContent>
    </Card>
  );
}
