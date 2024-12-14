import { useEffect } from "react";


// Define interfaces for message types
interface CompressedImage {
    data: string;
}

const Camera = ({ mqttSub, payload, status }: { mqttSub: Function; payload: any, status: boolean }) => {

    const cameraTopicName = 'plate_detection';

    useEffect(() => {
        if(!status) return;
        mqttSub(cameraTopicName);
    }, [status]);

    useEffect(() => {
        if (payload.topic == cameraTopicName) {
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
        <img id="video-stream" className="w-[640px] h-[480px]" />
    )
}

export default Camera;