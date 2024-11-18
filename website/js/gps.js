// Function to handle gps file input
// The file should be a CSV file of the topic /gnss_data
document.getElementById('csvFileInput').addEventListener('change', function (event) {
    const file = event.target.files[0];
    if (file) {
        const reader = new FileReader();

        reader.onload = function (e) {
            const csvData = e.target.result;
            const rows = csvData.split('\n'); // Split by newline to get each row

            const pathSegments = [];
            const colors = {
                5: 'green',  // RTK Float
                4: 'blue',   // RTK Fixed
                2: 'orange', // Differential GPS (no base fix)
                1: 'purple', // Autonomous (no base fix)
                0: 'red'     // No position
            };

            // Process CSV file
            rows.forEach((row, index) => {
                const columns = row.split(','); // The csv should be separated by commas

                // This topic has 6 items
                // 0 -> latitude
                // 1 -> longitude
                // 2 -> quality (from 0 to 6)
                if (columns.length >= 6 && !isNaN(columns[0]) && !isNaN(columns[1]) && !isNaN(columns[3])) {
                    // Parse the latitude and longitude as floats
                    const latitude = parseFloat(columns[0].trim());
                    const longitude = parseFloat(columns[1].trim());
                    const quality = parseInt(columns[3].trim(), 10);
                    
                    // Add the current point, color-coded by quality
                    if (index > 0) {
                        // Draw a line segment from the previous point to this one
                        const prevPoint = pathSegments[pathSegments.length - 1];
                        L.polyline([prevPoint.latlng, [latitude, longitude]], {
                            color: colors[quality] || 'gray', // Default color if quality is out of range
                            weight: 4,
                            opacity: 0.7
                        }).addTo(map);
                    }
                    // Save this point and its quality
                    pathSegments.push({
                        latlng: [latitude, longitude],
                        quality: quality
                    });
                }
            });
        };

        reader.readAsText(file); // Read the file as text
    } else {
        // It would be better to inform the user, for later
        console.log("No file selected.");
    }
});