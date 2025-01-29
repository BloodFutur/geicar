import type { Metadata } from 'next';
import * as React from 'react';

import { config } from '@/config';
import { CarInfo } from '@/components/dashboard/car-info/car-info';

export const metadata = { title: `Car Info | Dashboard | ${config.site.name}` } satisfies Metadata;

export default function Page(): React.JSX.Element { 
  return (
    <CarInfo/>
  );
}
