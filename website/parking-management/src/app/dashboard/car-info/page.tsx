import Stack from '@mui/material/Stack';
import Grid from '@mui/material/Unstable_Grid2';
import dayjs from 'dayjs';
import type { Metadata } from 'next';
import * as React from 'react';

import { Battery } from '@/components/dashboard/car-info/battery';
import Camera from '@/components/dashboard/car-info/camera';
import { Connection } from '@/components/dashboard/car-info/connection';
import { RunningTime } from '@/components/dashboard/car-info/running-time';
import { Status } from '@/components/dashboard/car-info/status';
import { type Check, SystemCheckTable } from '@/components/dashboard/car-info/system-check';
import { config } from '@/config';

export const metadata = { title: `Car Info | Dashboard | ${config.site.name}` } satisfies Metadata;

const checks = [
  {
    id: 'STM32F103 Communication',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'STM32L476 Communication',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'Nvidia Jetson Communication',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'Battery Level',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'Ultrasonic Sensors',
    status: 'Front Left: OK, Front Center: Out of range, Front Right: No Data | Rear Left: OK, Rear Center: OK, Rear Right: OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'GPS',
    status: 'RTK Fixed',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'IMU',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'Lidar',
    status: 'No Data',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
  {
    id: 'Camera',
    status: 'OK',
    createdAt: dayjs().subtract(2, 'hours').toDate(),
  },
] satisfies Check[];

export default function Page(): React.JSX.Element {
  return (
    <Stack spacing={3}>
      <Grid spacing={3} container>
        <Grid lg={3} sm={6} xs={12}>
          <Connection sx={{ height: '100%' }} value={false} />
        </Grid>
        <Grid lg={3} sm={6} xs={12}>
          <Status sx={{ height: '100%' }} value="Moving" />
        </Grid>
        <Grid lg={3} sm={6} xs={12}>
          <Battery sx={{ height: '100%' }} value={12.5} />
        </Grid>
        <Grid lg={3} sm={6} xs={12}>
          <RunningTime sx={{ height: '100%' }} value={45} />
        </Grid>
      </Grid>
      <SystemCheckTable checks={checks}/>
      <Camera/>
    </Stack>
  );
}
