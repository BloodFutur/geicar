import { useEffect, useRef, useState } from "react";
import * as L from "leaflet";

interface NavSatFix {
    latitude: number;
    longitude: number;
}


const Map = ({ mqttSub, payload, status }: { mqttSub: Function; payload: any, status: boolean }) => {
    const mapRef = useRef<L.Map | null>(null);
    const [gpsMessages, setGpsMessages] = useState<string[]>([]);
    const [parkingSpaces, setParkingSpaces] = useState<any[]>([]);
    const gpsTopicName = 'gps';
    const popup = useRef<L.Popup | null>(null);
    const parkingLayerRef = useRef<L.GeoJSON | null>(null);
    const [pathSegments, setPathSegments] = useState<any[]>([]);

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
        L.tileLayer('https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
            maxZoom: 22,
            subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
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
        legend.addTo(map);

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

    // Process the CSV file for GPS data
    const handleFileInput = (event: React.ChangeEvent<HTMLInputElement>) => {
        const files = event.target.files;
        if (files) {
            Array.from(files).forEach((file) => {
                const reader = new FileReader();

                reader.onload = function (e) {
                    const csvData = e.target?.result as string;
                    const rows = csvData.split('\n'); // Split by newline to get each row

                    const pathSegmentsTemp: any[] = [];

                    rows.forEach((row, index) => {
                        const columns = row.split(','); // The csv should be separated by commas

                        if (columns.length >= 6 && !isNaN(columns[0]) && !isNaN(columns[1]) && !isNaN(columns[3])) {
                            // Parse the latitude, longitude, and quality
                            const latitude = parseFloat(columns[0].trim());
                            const longitude = parseFloat(columns[1].trim());
                            const quality = parseInt(columns[3].trim(), 10);

                            // Add the current point, color-coded by quality
                            if (index > 0 && pathSegmentsTemp.length > 0) {
                                const prevPoint = pathSegmentsTemp[pathSegmentsTemp.length - 1];
                                // Draw a line segment from the previous point to this one
                                L.polyline([prevPoint.latlng, [latitude, longitude]], {
                                    color: colors[quality] || 'gray', // Default color if quality is out of range
                                    weight: 4,
                                    opacity: 0.7
                                }).addTo(mapRef.current!);
                            }

                            // Save this point and its quality
                            pathSegmentsTemp.push({
                                latlng: [latitude, longitude],
                                quality: quality
                            });
                        }
                    });

                    setPathSegments(pathSegmentsTemp); // Save path segments to state
                };

                reader.readAsText(file); // Read the file as text
            })
        } else {
            console.log("No file selected.");
        }
    };

    useEffect(() => {
        if (payload.topic == gpsTopicName) {
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
                color: "red",
                fillColor: "#f03",
                fillOpacity: 0.2,
                radius: 0.2,
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

        ways.forEach((way) => {
            const coordinates = way.nodes
                .map((nodeId: number) => {
                    const node = nodes.find((n) => n.id === nodeId);
                    return node ? [node.lat, node.lon] : null;
                })
                .filter((coord: any) => coord !== null);

            if (coordinates.length) {
                const poly = L.polygon(coordinates as L.LatLngExpression[]);
                poly.on("click", () => {
                    const bounds = poly.getBounds();
                    popup.current?.setLatLng(bounds.getCenter());
                    popup.current?.setContent("FREE");
                    mapRef.current?.openPopup(popup.current);
                });
                parkingLayerRef.current?.addLayer(poly);
            }
        });
    }, [parkingSpaces]);

    // Fetch parking data on mount
    useEffect(() => {
        fetchParkingData();
    }, []);

    return (
        <section id="map-section" aria-label="Map Display">
            <label htmlFor="csvFileInput">Upload CSV file:</label>
            <input type="file" id="csvFileInput" accept=".csv" onChange={handleFileInput} multiple />
            <div id="map"></div>
            <section id="gps-messages" aria-label="GPS Messages">
                <h3>Received GPS Coordinates: {gpsMessages.length}</h3>
                <ul>
                    {gpsMessages.map((message, index) => (
                        <li key={index}>{message}</li>
                    ))}
                </ul>
            </section>
        </section>
    )
}

export default Map;