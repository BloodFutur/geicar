'use client';

import { useMqtt } from "@/contexts/mqtt-context";
import { Card, CardHeader, Divider, Typography } from "@mui/material";
import { useEffect } from "react";


// Define interfaces for message types
interface CompressedImage {
    data: string;
}

function Camera() {

    const { status, payload, mqttSub } = useMqtt();
    
    const cameraTopicName = 'plate_detection';

    useEffect(() => {
        if(!status) return;
        mqttSub(cameraTopicName);
    }, [status]);

    useEffect(() => {
        if (payload != null && payload.topic == cameraTopicName) {
            try {
                const plateDetectionMsg = JSON.parse(payload.message.toString());
                handlePlateDetection(plateDetectionMsg)
            } catch (error) {
                console.error('Error parsing NavSatFix message:', error);
            }
        }
    }, [payload])


    const handlePlateDetection = (message: CompressedImage) => {
        const videoStream = document.getElementById(
            "video-stream"
        ) as HTMLImageElement | null;

        if (videoStream) {
            const base64Image = `data:image/jpeg;base64,${message.data}`;
            videoStream.src = base64Image;
        } else {
            console.error("Video stream element not found!");
        }
    };

    return (
        <Card>
            <CardHeader title="Camera" />
            <Divider />
            <img id="video-stream" style={{ width: '100%', aspectRatio: '720 / 480', padding:'4px' }} />
        </Card>
    )
}

export default Camera;