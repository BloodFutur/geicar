import * as React from 'react';
import Box from '@mui/material/Box';
import Button from '@mui/material/Button';
import Card from '@mui/material/Card';
import CardActions from '@mui/material/CardActions';
import CardHeader from '@mui/material/CardHeader';
import Chip from '@mui/material/Chip';
import Divider from '@mui/material/Divider';
import type { SxProps } from '@mui/material/styles';
import Table from '@mui/material/Table';
import TableBody from '@mui/material/TableBody';
import TableCell from '@mui/material/TableCell';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import { ArrowRight as ArrowRightIcon } from '@phosphor-icons/react/dist/ssr/ArrowRight';
import dayjs from 'dayjs';

const statusMap = {
  'OK': { label: 'OK', color: 'success' },
  'RTK Fixed': { label: 'RTK Fixed', color: 'success' },
  'RTK Float': { label: 'RTK Float', color: 'warning' },
  'No Fix': { label: 'No Fix', color: 'error' },
  'Autonomous GNSS fix': { label: 'Autonomous GNSS fix', color: 'success' },
  'Differential GNSS fix': { label: 'Differential GNSS fix', color: 'success' },
  'Estimated/dead reckoning fix': { label: 'Estimated/dead reckoning fix', color: 'warning' },
  'No Data': { label: 'No Data', color: 'error' },
} as const;

export interface Check {
  id: string;
  status: string;
  createdAt: Date;
}

export interface SystemCheckTableProps {
  checks?: Check[];
  sx?: SxProps;
}

export function SystemCheckTable({ checks = [], sx }: SystemCheckTableProps): React.JSX.Element {
  return (
    <Card sx={sx}>
      <CardHeader title="System Check Report" />
      <Divider />
      <Box sx={{ overflowX: 'auto' }}>
        <Table sx={{ minWidth: 200 }}>
          <TableHead>
            <TableRow>
              <TableCell sx={{ minWidth: { lg: 250 } }}>Component</TableCell>
              <TableCell sx={{ minWidth: { lg: 250 } }} sortDirection="desc">Date</TableCell>
              <TableCell>Status</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
            {checks.map((check) => {
              return (
                <TableRow hover key={check.id}>
                  <TableCell>{check.id}</TableCell>
                  <TableCell>{dayjs(check.createdAt).format('dddd, MMMM D, YYYY h:mm')}</TableCell>
                  <TableCell>
                    {check.id === 'Ultrasonic Sensors' ? (
                      // Separate the status string of the Ultrasonic Sensors by ',' or '|'
                      check.status.split(/,|\|/).map((status, index) => {
                        const trimmedStatus = status.split(':')[1].trim();
                        let { label, color } = statusMap[trimmedStatus as 'OK' | 'No Data'] ?? { label: trimmedStatus, color: 'warning' };
                        label = `${status.split(':')[0].trim()  }: ${  label}`;
                        return (
                          <Chip color={color} label={label} size="small" sx={{my: 0.5, mr: 0.5}} />
                        );
                      })
                    ) : (
                      (() => {
                        const { label, color } = statusMap[check.status as 'OK' | 'No Data'] ?? { label: check.status, color: 'default' };
                        return <Chip color={color} label={label} size="small"/>;
                      })()
                    )}
                  </TableCell>
                </TableRow>
              );
            })}
          </TableBody>
        </Table>
      </Box>
    </Card>
  );
}
