// Make a leaflet map centered on INSA Toulouse
var map = L.map('map').setView([43.5705915, 1.4663547, 84], 19);

// Add Google Maps tiles to the map (best one found to see parking lots with a good zooming level)
var googleSat = L.tileLayer('https://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
    maxZoom: 22,
    subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
}).addTo(map);


// Add legend to GPS positions
// Colors are associated to the colors of colors variable in `gps.js` 
var legend = L.control({ position: "bottomleft" });
legend.onAdd = function (map) {
    var div = L.DomUtil.create("div", "legend");
    div.innerHTML += "<h4>GPS Quality</h4>";
    div.innerHTML += '<i style="background: green"></i><span>RTK Float</span><br>';
    div.innerHTML += '<i style="background: blue"></i><span>RTK Fixed</span><br>';
    div.innerHTML += '<i style="background: orange"></i><span>Diff GPS(without base fix)</span><br>';
    div.innerHTML += '<i style="background: purple"></i><span>Normal(without base fix)</span><br>';
    div.innerHTML += '<i style="background: red"></i><span>No Position</span><br>';
    return div;
};
legend.addTo(map);


var parking_nodes = [];
var parking_ways = [];

function storeParkingData(data) {
    data.elements.forEach(element => {
        if (element.type === "node") {
            parking_nodes.push(element);
        } else if (element.type === "way") {
            parking_ways.push(element);
        }
    });
}

// Function to fetch parking data from Overpass API
function fetchParkingData() {
    var overpassQuery = `
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

    var url = "https://overpass-api.de/api/interpreter?data=" + encodeURIComponent(overpassQuery);

    fetch(url)
        .then(response => response.json())
        .then(data => {
            storeParkingData(data);
            addParkingToMap();
        })
        .catch(error => console.error('Error fetching parking data:', error));
}

var parking_spaces_polygons = [];
popup = new L.Popup();
function addParkingToMap() {
    var parkingLayer = L.geoJSON(null, {
        style: function (feature) {
            return { color: "blue" }; // Styling for parking areas
        },
        pointToLayer: function (feature, latlng) {
            return L.marker(latlng, { icon: L.icon({ iconUrl: 'parking-icon.png', iconSize: [20, 20] }) });
        }
    });
    parking_ways.forEach(element => {
        // Get all nodes of the ways
        // Get their coordinates
        // Make a polygone
        var nodes_id = element.nodes;
        var coordinates = nodes_id
            .map(id => parking_nodes.find(node => node.id == id)) // Find the node by ID
            .filter(node => node) // Ensure no `undefined` values are included
            .map(node => [node.lat, node.lon]);
        let poly = L.polygon(coordinates)
        poly.on("click", function (e) {
            var bounds = poly.getBounds();
            var popupContent = "FREE";
            popup.setLatLng(bounds.getCenter());
            popup.setContent(popupContent);
            map.openPopup(popup);
        });
        parking_spaces_polygons.push(coordinates);
        parkingLayer.addLayer(poly);
    })
    parkingLayer.addTo(map);
}


// Fetch parking data
fetchParkingData();
