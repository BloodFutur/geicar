import Grid from '@mui/material/Unstable_Grid2';
import type { Metadata } from 'next';
import * as React from 'react';

import { AverageParkedCars } from '@/components/dashboard/overview/average-parked-cars';
import { AverageParkedTime } from '@/components/dashboard/overview/average-parked-time';
import { CarMode } from '@/components/dashboard/overview/car-mode';
import { ParkedCars } from '@/components/dashboard/overview/parked-cars';
import { ParkingHourlyUsage } from '@/components/dashboard/overview/parking-hourly-usage';
import { TotalDistance } from '@/components/dashboard/overview/total-distance';
import { config } from '@/config';

export const metadata = { title: `Overview | Dashboard | ${config.site.name}` } satisfies Metadata;

export default function Page(): React.JSX.Element {
  return (
    <Grid container spacing={3}>
      <Grid lg={3} sm={6} xs={12}>
        <ParkedCars sx={{ height: '100%' }} value={100} total={150} />
      </Grid>
      <Grid lg={3} sm={6} xs={12}>
        <AverageParkedCars sx={{ height: '100%' }} value="85" />
      </Grid>
      <Grid lg={3} sm={6} xs={12}>
        <AverageParkedTime sx={{ height: '100%' }} value="3h" />
      </Grid>
      <Grid lg={3} sm={6} xs={12}>
        <TotalDistance sx={{ height: '100%' }} value="15km" />
      </Grid>
      <Grid lg={8} xs={12}>
        <ParkingHourlyUsage
          chartSeries={[
            { name: 'Daily average', data: [18, 40, 28, 52, 33, 60, 34, 40, 52, 31, 40, 28] },

            // { name: 'Last year', data: [12, 11, 4, 6, 2, 9, 9, 10, 11, 12, 13, 13] },
          ]}
          sx={{ height: '100%' }}
        />
      </Grid>
      <Grid lg={4} md={6} xs={12}>
        <CarMode chartSeries={[63, 27]} labels={['Manual', 'Autonomous']} sx={{ height: '100%' }} />
      </Grid>
    </Grid>
  );
}
