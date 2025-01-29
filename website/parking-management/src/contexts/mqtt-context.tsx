'use client';

import * as React from 'react';

import mqtt, { MqttClient } from 'mqtt';


export interface MqttContextValue {
  mqttClient: MqttClient | null;
  status: boolean;
  payload: { topic: string; message: string };
  mqttSub: (topic: string) => void;
}

export const MqttContext = React.createContext<MqttContextValue | undefined>(undefined);

export interface MqttProviderProps {
  children: React.ReactNode;
}

export function MqttProvider({ children }: MqttProviderProps): React.JSX.Element {
  const [status, setStatus] = React.useState<boolean>(false);
  const mqttClientRef = React.useRef<MqttClient | null>(null);
  const [payload, setPayload] = React.useState<{ topic: string; message: string } | null>(null);

  // console.log('MQTT Broker URL:', process.env.NEXT_PUBLIC_MQTT_BROKER_URL);
  // console.log('MQTT Client ID:',  process.env.NEXT_PUBLIC_MQTT_CLIENT_ID);

  React.useEffect(() => {
    // Set up the MQTT client connection
    const mqttClient = mqtt.connect(process.env.NEXT_PUBLIC_MQTT_BROKER_URL || '', {
      clientId: process.env.NEXT_PUBLIC_MQTT_CLIENT_ID || 'react-client', // Unique client ID
      username: process.env.NEXT_PUBLIC_MQTT_USERNAME || '',
      password: process.env.NEXT_PUBLIC_MQTT_PASSWORD || ''
    });

    mqttClientRef.current = mqttClient;

    // Event handler when connected to the broker
    mqttClient.on('connect', () => {
      setStatus(true);
    });

    mqttClient.on('message', (topic, message) => {
      const payload = { topic, message: message.toString() };
      setPayload(payload);
    });

    mqttClient.on('error', (err) => {
      console.error('Connection error: ', err);
      mqttClient.end();
    });

    // Clean up the MQTT client connection when the component unmounts
    return () => {
      if (mqttClient) {
        mqttClient.end();
      }
    };
  }, []);

  const mqttSub = (topic: string) => {
    if (mqttClientRef.current) {
      mqttClientRef.current.subscribe(topic, (error) => {
        if (error) {
          console.log(`Subscribe to topic ${topic} error`, error);
          return;
        }
        console.log(`Subscribe to topic: ${topic}`);
      });
    }
  };


  return (
    <MqttContext.Provider value={{ mqttClient: mqttClientRef.current, status, payload, mqttSub }}>
      {children}
    </MqttContext.Provider>
  );
}

export const useMqtt = (): MqttContextValue => {
  const context = React.useContext(MqttContext);
  if (context === undefined) {
    throw new Error('useMqtt must be used within a MqttProvider');
  }
  return context;
};