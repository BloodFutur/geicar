import Grid from '@mui/material/Unstable_Grid2';
import type { Metadata } from 'next';
import * as React from 'react';

import { AverageParkedTime } from '@/components/dashboard/overview/average-parked-time';
import { ParkedCars } from '@/components/dashboard/overview/parked-cars';
import { LongestParkedTime } from '@/components/dashboard/parking-management/longest-parked-time';
import ParkingManagementMap from '@/components/dashboard/parking-management/pm-map';
import { config } from '@/config';

export const metadata = { title: `Parking Management | Dashboard | ${config.site.name}` } satisfies Metadata;

export default function Page(): React.JSX.Element {

  return (
    <Grid container spacing={3}>
      <Grid lg={4} sm={6} xs={12}>
        <ParkedCars sx={{ height: '100%' }} value={100} total={150} />
      </Grid>
      <Grid lg={4} sm={6} xs={12}>
        <AverageParkedTime sx={{ height: '100%' }} value="3h" />
      </Grid>
      <Grid lg={4} sm={6} xs={12}>
        <LongestParkedTime sx={{ height: '100%' }} value="85" />
      </Grid>
      <ParkingManagementMap />
    </Grid>
  );
}
