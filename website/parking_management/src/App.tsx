import { useEffect, useRef, useState } from "react";
import Map from "./components/Map";
import Camera from "./components/Camera";
import mqtt, { MqttClient } from 'mqtt';
import VehicleStatus from "./components/VehicleStatus";


function App() {

  const [status, setStatus] = useState<boolean>(false);
  const mqtt_client_ref = useRef<MqttClient | null>(null);
  const [payload, setPayload] = useState({})

  useEffect(() => {
    // Set up the MQTT client connection
    const mqtt_client = mqtt.connect('ws://147.79.101.94:9001', {
      clientId: 'react-client', // Unique client ID
      username: 'geicar',
      password: 'geicar'
    });

    mqtt_client_ref.current = mqtt_client;

    // Event handler when connected to the broker
    mqtt_client.on('connect', () => {
      //console.log('Connected to MQTT broker');
      setStatus(true);
    });

    mqtt_client.on('message', (topic, message) => {
      const payload = { topic, message: message.toString() }
      setPayload(payload)
      //console.log(`received message: ${message} from topic: ${topic}`)
    })

    mqtt_client.on('error', (err) => {
      console.error('Connection error: ', err);
      mqtt_client.end();
    });

    // Clean up the MQTT client connection when the component unmounts
    // return () => {
    //   if (mqtt_client) {
    //     mqtt_client.end();
    //   }
    // };
  }, []);


  const mqttSub = (topic: string) => {
    if (mqtt_client_ref.current) {
      mqtt_client_ref.current.subscribe(topic, (error) => {
        if (error) {
          console.log(`Subscribe to topic ${topic} error`, error);
          return;
        }
        console.log(`Subscribe to topic: ${topic}`);
      });
    }
  }


  return (
    <>
      <div id="test">
        <header>
          <h1>Park Patroller</h1>
        </header>

        <main id="main-container">
          <section id="data-section" aria-label="Instructions and Connection Status">

            <p>Connection: <span id="status" className={status ? 'connected' : 'closed'}>{status ? "Connected" : "Closed"}</span></p>
            <VehicleStatus mqttSub={mqttSub} payload={payload} status={status}></VehicleStatus>
          </section>

          <Map mqttSub={mqttSub} payload={payload} status={status}></Map>
        </main>
        <Camera mqttSub={mqttSub} payload={payload} status={status}></Camera>
      </div>
    </>
  )
}

export default App
