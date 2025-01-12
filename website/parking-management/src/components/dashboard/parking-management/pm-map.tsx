'use client';

import { useEffect, useRef, useState, memo } from "react";
import * as L from "leaflet";
import './pm-map.css';
import { useMqtt } from "@/contexts/mqtt-context";

interface NavSatFix {
    latitude: number;
    longitude: number;
}


function ParkingManagementMap() {
    const { status, payload, mqttSub } = useMqtt();

    const mapRef = useRef<L.Map | null>(null);
    const [gpsMessages, setGpsMessages] = useState<string[]>([]);
    const [parkingSpaces, setParkingSpaces] = useState<any[]>([]);
    const gpsTopicName = 'gps';
    const popup = useRef<L.Popup | null>(null);
    const parkingLayerRef = useRef<L.GeoJSON | null>(null);
    const [pathSegments, setPathSegments] = useState<any[]>([]);
    const addedParkingSpaces = new Set();

    const colors = {
        5: 'green',  // RTK Float
        4: 'blue',   // RTK Fixed
        2: 'orange', // Differential GPS (no base fix)
        1: 'purple', // Autonomous (no base fix)
        0: 'red'     // No position
    };

    useEffect(() => {
        if (!status) return;
        mqttSub(gpsTopicName);
    }, [status]);

    // Initialize Leaflet map
    useEffect(() => {
        const map = L.map("map").setView([43.5705915, 1.4663547, 84], 19);

        // Add Google Maps tiles to the map (best one found to see parking lots with a good zooming level)
        // L.tileLayer('https://{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
        //     maxZoom: 22,
        //     subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
        // }).addTo(map);
        L.tileLayer('https://{s}.tile.openstreetmap.fr/osmfr/{z}/{x}/{y}.png', {
            maxZoom: 22,
            subdomains: ['a', 'b', 'c']
        }).addTo(map);
        // Add legend to GPS positions
        // Colors are associated to the colors of colors variable in `gps.js` 
        const legend = new L.Control({ position: "bottomleft" });
        legend.onAdd = function () {
            const div = L.DomUtil.create("div", "legend");
            div.innerHTML += "<h4>GPS Quality</h4>";
            div.innerHTML += '<i style="background: green"></i><span>RTK Float</span><br>';
            div.innerHTML += '<i style="background: blue"></i><span>RTK Fixed</span><br>';
            div.innerHTML += '<i style="background: orange"></i><span>Diff GPS(without base fix)</span><br>';
            div.innerHTML += '<i style="background: purple"></i><span>Normal(without base fix)</span><br>';
            div.innerHTML += '<i style="background: red"></i><span>No Position</span><br>';
            return div;
        };
        //legend.addTo(map);

        mapRef.current = map;

        popup.current = new L.Popup();

        parkingLayerRef.current = L.geoJSON(null, {
            style: () => ({ color: "blue" }),
            pointToLayer: (_, latlng) => L.marker(latlng, { icon: L.icon({ iconUrl: "parking-icon.png", iconSize: [20, 20] }) }),
        }).addTo(map);

        return () => {
            map.remove();
        };
    }, []);

    useEffect(() => {
        if (payload != null && payload.topic == gpsTopicName) {
            try {
                // Parse the NavSatFix message
                //console.log(payload.message);
                const navSatFix = JSON.parse(payload.message.toString());
                handleGPSMessage(navSatFix)
            } catch (error) {
                console.error('Error parsing NavSatFix message:', error);
            }
        }
    }, [payload])


    const handleGPSMessage = (message: NavSatFix) => {
        // Update messages
        const coords = `${message.latitude}; ${message.longitude}`;
        setGpsMessages((prevMessages) => [...prevMessages, coords]);

        // Add to map
        if (mapRef.current) {
            L.circle([message.latitude, message.longitude], {
                color: "blue",
                fillColor: "#30f",
                fillOpacity: 0.2,
                radius: 0.3,
            }).addTo(mapRef.current);
        }
    };

    // Fetch parking data from Overpass API
    const fetchParkingData = async () => {
        const overpassQuery = `
            [out:json];
            (
              node["amenity"="parking_space"](around:100,43.5705915,1.4663547);
              way["amenity"="parking_space"](around:100,43.5705915,1.4663547);
              relation["amenity"="parking_space"](around:100,43.5705915,1.4663547);
            );
            out body;
            >;
            out skel qt;
        `;

        const url = `https://overpass-api.de/api/interpreter?data=${encodeURIComponent(overpassQuery)}`;
        try {
            const response = await fetch(url);
            const data = await response.json();
            setParkingSpaces(data.elements || []);
        } catch (error) {
            console.error("Error fetching parking data:", error);
        }
    };

    // Add parking data to the map
    useEffect(() => {
        if (!parkingSpaces.length || !mapRef.current || !parkingLayerRef.current) return;


        const nodes = parkingSpaces.filter((el) => el.type === "node");
        const ways = parkingSpaces.filter((el) => el.type === "way");
        console.log('Nodes:', nodes.length, 'Ways:', ways.length);
        ways.forEach((way) => {
            if (addedParkingSpaces.has(way.id)) return;

            const coordinates = way.nodes
                .map((nodeId: number) => {
                    const node = nodes.find((n) => n.id === nodeId);
                    return node ? [node.lat, node.lon] : null;
                })
                .filter((coord: any) => coord !== null);

            if (coordinates.length) {
                const poly = L.polygon(coordinates as L.LatLngExpression[]);
                const taken = Math.random() < 0.7;
                const color = taken ? '#FF0000' : '#00FF00';
                poly.setStyle({fillColor: color, color});
                poly.on("click", () => {
                    const bounds = poly.getBounds();
                    popup.current?.setLatLng(bounds.getCenter());
                    const content = ` \
                    <h3>Parking Space</h3> \
                    <p>${  taken ? 'Taken' : 'Free'  }</p> \
                    <p>${  taken ? 'AA-123-BB' : ''  }</p> \
                    `;
                    popup.current?.setContent(content);
                    mapRef.current?.openPopup(popup.current);
                });
                parkingLayerRef.current?.addLayer(poly);
                addedParkingSpaces.add(way.id);
            }
        });
        console.log('Added parking spaces:', addedParkingSpaces.size);
    }, [parkingSpaces]);

    // Fetch parking data on mount
    useEffect(() => {
        console.log('Fetching parking data');
        fetchParkingData();
        console.log('Component mounted');

        return () => {
            console.log('Component unmounted');
        };
    }, []);

    return (
        <section id="map-section" aria-label="Map Display">
            <div id="map" />
        </section>
    )
}

export default ParkingManagementMap;