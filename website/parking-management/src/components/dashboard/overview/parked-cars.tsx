import * as React from 'react';
import Avatar from '@mui/material/Avatar';
import Card from '@mui/material/Card';
import CardContent from '@mui/material/CardContent';
import Stack from '@mui/material/Stack';
import type { SxProps } from '@mui/material/styles';
import Typography from '@mui/material/Typography';
import { LetterCircleP as LetterCirclePIcon } from '@phosphor-icons/react/dist/ssr/LetterCircleP';
import LinearProgress from '@mui/material/LinearProgress';

export interface ParkedCarsProps {
    sx?: SxProps;
    value: number;
    total: number
}

export function ParkedCars({ value, total, sx }: ParkedCarsProps): React.JSX.Element {
    return (
        <Card sx={sx}>
            <CardContent>
                <Stack direction="row" sx={{ alignItems: 'flex-start', justifyContent: 'space-between' }} spacing={3}>
                    <Stack spacing={1}>
                        <Typography color="text.secondary" variant="overline">
                            Parked Cars
                        </Typography>
                        <Typography variant="h4">{value}/{total}</Typography>
                    </Stack>
                    <Avatar sx={{ backgroundColor: 'var(--mui-palette-primary-main)', height: '56px', width: '56px' }}>
                        <LetterCirclePIcon fontSize="var(--icon-fontSize-lg)" />
                    </Avatar>
                </Stack>
                <div>
                    <LinearProgress 
                        value={(value / total) * 100} 
                        variant="determinate" 
                        color={(value / total) > 0.9 ? 'error' : (value / total < 0.6 ? 'success' : 'warning')} 
                    />
                </div>
            </CardContent>
        </Card>
    );
}
