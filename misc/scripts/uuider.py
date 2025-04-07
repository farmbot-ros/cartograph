import json
import uuid

# Load the GeoJSON file
with open("../config/wur.json", "r") as file:
    geojson_data = json.load(file)

# Add UUID and ID as properties
for feature in geojson_data.get("features", []):
    if "properties" not in feature:
        feature["properties"] = {}  # Ensure properties exist
    feature["properties"]["uuid"] = str(uuid.uuid4())  # Add unique UUID
    if "id" in feature:
        feature["properties"]["id"] = feature["id"]  # Add existing ID to properties


# Save the modified GeoJSON
with open("../config/wur.json", "w") as file:
    json.dump(geojson_data, file, indent=2)

print("UUIDs added to each feature.")

