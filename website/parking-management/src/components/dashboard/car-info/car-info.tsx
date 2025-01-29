'use client';

import Stack from '@mui/material/Stack';
import Grid from '@mui/material/Unstable_Grid2';
import dayjs from 'dayjs';
import * as React from 'react';
import { useEffect } from "react";

import { Battery } from '@/components/dashboard/car-info/battery';
import Camera from '@/components/dashboard/car-info/camera';
import { Connection } from '@/components/dashboard/car-info/connection';
import { RunningTime } from '@/components/dashboard/car-info/running-time';
import { Status } from '@/components/dashboard/car-info/status';
import { type Check, SystemCheckTable } from '@/components/dashboard/car-info/system-check';
import { useMqtt } from '@/contexts/mqtt-context';


const defaultChecks = [
    {
        id: 'STM32F103 Communication',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'STM32L476 Communication',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'Nvidia Jetson Communication',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'Battery Level',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'Ultrasonic Sensors',
        status: 'Front Left: OK, Front Center: Out of range, Front Right: No Data | Rear Left: OK, Rear Center: OK, Rear Right: OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'GPS',
        status: 'RTK Fixed',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'IMU',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'Lidar',
        status: 'No Data',
        createdAt: dayjs().toDate(),
    },
    {
        id: 'Camera',
        status: 'OK',
        createdAt: dayjs().toDate(),
    },
] satisfies Check[];

interface SystemCheckMessage {
    request: boolean;
    response: boolean;
    report: boolean;
    print: boolean;

    jetson: boolean;
    l476: boolean;
    f103: boolean;

    comm_jetson: "OK" | "Failed";
    comm_l476: "OK" | "Failed";
    comm_f103: "OK" | "Failed";

    battery: "OK" | "Low Voltage" | "High Voltage" | "No data";

    ultrasonics: Array<"OK" | "Out of range" | "No data">;

    gps: "No Fix" | "Autonomous GNSS fix" | "Differential GNSS fix" | "RTK fixed" | "RTK float" | "Estimated/dead reckoning fix" | "No data";

    imu: "OK" | "No data";
    lidar: "OK" | "No data";
    camera: "OK" | "No data";
};

interface GeneralDataMessage {
    battery_level: number;
    temperature: number;
    pressure: number;
    humidity: number;
};

interface VehicleStatusMessage {
    status: string;
};

interface VehicleUptimeMessage {
    uptime: string;
};

function transformSystemCheck(message: SystemCheckMessage, currentTime: Date): Check[] {
    return [
        {
            id: 'STM32F103 Communication',
            status: message.comm_f103,
            createdAt: currentTime,
        },
        {
            id: 'STM32L476 Communication',
            status: message.comm_l476,
            createdAt: currentTime,
        },
        {
            id: 'Nvidia Jetson Communication',
            status: message.comm_jetson,
            createdAt: currentTime,
        },
        {
            id: 'Battery Level',
            status: message.battery,
            createdAt: currentTime,
        },
        {
            id: 'Ultrasonic Sensors',
            status: `Front Left: ${message.ultrasonics[0]}, Front Center: ${message.ultrasonics[1]}, Front Right: ${message.ultrasonics[2]} | Rear Left: ${message.ultrasonics[3]}, Rear Center: ${message.ultrasonics[4]}, Rear Right: ${message.ultrasonics[5]}`,
            createdAt: currentTime,
        },
        {
            id: 'GPS',
            status: message.gps,
            createdAt: currentTime,
        },
        {
            id: 'IMU',
            status: message.imu,
            createdAt: currentTime,
        },
        {
            id: 'Lidar',
            status: message.lidar,
            createdAt: currentTime,
        },
        {
            id: 'Camera',
            status: message.camera,
            createdAt: currentTime,
        }
    ] satisfies Check[];
}

function updateChecks(currentChecks: Check[], message: SystemCheckMessage): Check[] {
    const currentTime = dayjs().toDate();
    const newChecks: Check[] = transformSystemCheck(message, currentTime);

    return currentChecks.map((check: Check, index) => {
        const newCheck = newChecks[index];

        // if status is different, update the check
        if (newCheck.status !== check.status) {
            return {
                ...newCheck,
                status: newCheck.status,
                createdAt: currentTime,
            };
        }
        return check; // if status is the same, return the old check
    });
}

export function CarInfo(): React.JSX.Element {
    const [checks, setChecks] = React.useState<Check[]>(defaultChecks);
    const [batteryLevel, setBatteryLevel] = React.useState<number>(0);
    const [carStatus, setCarStatus] = React.useState<string>('Stopped');
    const [runningTime, setRunningTime] = React.useState<number>(0);
    const { status, payload, mqttSub } = useMqtt();

    const systemCheckTopic = 'system_check';
    const generalDataTopic = 'general_data';
    const vehicleStatusTopic = 'vehicle/status';
    const vehicleUptimeTopic = 'vehicle/uptime';

    useEffect(() => {
        if (!status) return;
        mqttSub(systemCheckTopic);
        mqttSub(generalDataTopic);
        mqttSub(vehicleStatusTopic);
        mqttSub(vehicleUptimeTopic);
    }, [status]);

    useEffect(() => {
        if (payload?.topic == systemCheckTopic) {
            try {
                const message: SystemCheckMessage = JSON.parse(payload.message.toString());

                if (message.report) {
                    const updatedChecks = updateChecks(checks, message);
                    setChecks(updatedChecks);
                    console.log('Updated checks:', updatedChecks);
                }

            } catch (error) {
                console.error('Error parsing system check message:', error);
            }
        }

        if (payload?.topic == generalDataTopic) {
            try {
                const message: GeneralDataMessage = JSON.parse(payload.message.toString());
                setBatteryLevel(message.battery_level);
            }
            catch (error) {
                console.error('Error parsing general data message:', error);
            }
        }
        if (payload?.topic == vehicleStatusTopic) {
            try {
                const message: string = payload.message.toString();
                setCarStatus(message);
            }
            catch (error) {
                console.error('Error parsing vehicle status message:', error);
            }
        }
        if (payload?.topic == vehicleUptimeTopic) {
            try {
                // There are double quotes and backslashes in the message, so we need to parse it twice
                const message = JSON.parse(JSON.parse(payload.message));
                
                const [hours, minutes, seconds] = message.uptime.split(':').map(Number);
                const uptimeInSeconds = (hours * 3600) + (minutes * 60) + seconds;
                const uptimeInMinutes = uptimeInSeconds / 60;
                setRunningTime(uptimeInMinutes);
            }
            catch (error) {
                console.error('Error parsing vehicle uptime message:', error);
            }
        }
    }, [payload]);

    return (
        <Stack spacing={3}>
            <Grid spacing={3} container>
                <Grid lg={3} sm={6} xs={12}>
                    <Connection sx={{ height: '100%' }} value={false} />
                </Grid>
                <Grid lg={3} sm={6} xs={12}>
                    <Status sx={{ height: '100%' }} value={carStatus} />
                </Grid>
                <Grid lg={3} sm={6} xs={12}>
                    <Battery sx={{ height: '100%' }} value={batteryLevel} />
                </Grid>
                <Grid lg={3} sm={6} xs={12}>
                    <RunningTime sx={{ height: '100%' }} value={runningTime} />
                </Grid>
            </Grid>
            <SystemCheckTable checks={checks} />
            <Camera />
        </Stack>
    );
}
