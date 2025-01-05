import { useEffect, useState } from "react";


// Define interfaces for message types
interface VehicleStatusMessage {
    status: string;
}

const VehicleStatus = ({ mqttSub, payload, status }: { mqttSub: Function; payload: any, status: boolean }) => {

    const [vehicleStatus, setVehicleStatus] = useState<string>("Disconnected");
    const vehicleStatusTopicName = 'vehicle/status';

    useEffect(() => {
        if(!status) return;
        mqttSub(vehicleStatusTopicName);
    }, [status]);

    useEffect(() => {
        if (payload.topic == vehicleStatusTopicName) {
            try {
                const statusMsg = JSON.parse(payload.message.toString());
                handleStatus(statusMsg)
            } catch (error) {
                console.error('Error parsing NavSatFix message:', error);
            }
        }
    }, [payload])


    const handleStatus = (message: VehicleStatusMessage) => {
        setVehicleStatus(message.status)
    };

    return (
        <p> The vehicle is {vehicleStatus}</p>
    )
}

export default VehicleStatus;