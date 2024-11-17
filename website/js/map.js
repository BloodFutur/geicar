// Make a leaflet map centered on INSA Toulouse
var map = L.map('map').setView([43.5705915, 1.4663547, 84], 19);

// Add Google Maps tiles to the map (best one found to see parking lots with a good zooming level)
var googleSat = L.tileLayer('http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
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