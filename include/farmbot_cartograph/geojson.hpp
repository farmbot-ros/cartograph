#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogr_geometry.h>
#include <ogrsf_frmts.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "farmbot_interfaces/msg/geo_json.hpp"
#include "farmbot_interfaces/msg/feature.hpp"
#include "farmbot_interfaces/msg/geometry.hpp"
#include "farmbot_interfaces/msg/cordinates.hpp"
#include "farmbot_interfaces/msg/cordinate.hpp"
#include "farmbot_interfaces/msg/propertie.hpp"

namespace geojson_parser{
    using farmbot_interfaces::msg::GeoJson;
    using farmbot_interfaces::msg::Feature;
    using farmbot_interfaces::msg::Geometry;
    using farmbot_interfaces::msg::Cordinates;
    using farmbot_interfaces::msg::Cordinate;
    using farmbot_interfaces::msg::Propertie;

    static std::vector<Propertie> readProperties(OGRFeature* feature){
        std::vector<Propertie> properties;
        if (!feature) {
            return properties;
        }
        int fieldCount = feature->GetFieldCount();
        for (int i = 0; i < fieldCount; ++i) {
            if (feature->IsFieldSetAndNotNull(i)) {
            Propertie prop;
            OGRFieldDefn* fieldDefn = feature->GetDefnRef()->GetFieldDefn(i);
            prop.key = fieldDefn->GetNameRef();
            switch (fieldDefn->GetType()) {
                case OFTString:
                prop.value = feature->GetFieldAsString(i);
                break;
                case OFTInteger:
                case OFTInteger64:
                prop.value = std::to_string(feature->GetFieldAsInteger64(i));
                break;
                case OFTReal:
                prop.value = std::to_string(feature->GetFieldAsDouble(i));
                break;
                default: {
                const char* rawVal = feature->GetFieldAsString(i);
                prop.value = rawVal ? rawVal : "";
                } break;
            }
            properties.push_back(prop);
            }
        }
        return properties;
    }

    static Geometry convertOGRGeometry(OGRGeometry* geometry) {
        Geometry geometry_msg;
        if (!geometry) {
            return geometry_msg;
        }
        OGRwkbGeometryType gType = wkbFlatten(geometry->getGeometryType());
        switch (gType) {
            case wkbPoint: {
            geometry_msg.type = Geometry::POINT;
            OGRPoint* pt = geometry->toPoint();
            if (pt) {
                Cordinate c;
                c.x = pt->getX();
                c.y = pt->getY();
                c.z = pt->getZ();
                geometry_msg.cordinates.points.push_back(c);
            }
            } break;
            case wkbLineString: {
            geometry_msg.type = Geometry::LINESTRING;
            OGRLineString* ls = geometry->toLineString();
            if (ls) {
                int numPoints = ls->getNumPoints();
                for (int i = 0; i < numPoints; ++i) {
                Cordinate c;
                c.x = ls->getX(i);
                c.y = ls->getY(i);
                c.z = ls->getZ(i);
                geometry_msg.cordinates.points.push_back(c);
                }
            }
            } break;
            case wkbPolygon: {
            geometry_msg.type = Geometry::POLYGON;
            OGRPolygon* poly = geometry->toPolygon();
            if (poly) {
                OGRLinearRing* ring = poly->getExteriorRing();
                if (ring) {
                int numPoints = ring->getNumPoints();
                for (int i = 0; i < numPoints; ++i) {
                    Cordinate c;
                    c.x = ring->getX(i);
                    c.y = ring->getY(i);
                    c.z = ring->getZ(i);
                    geometry_msg.cordinates.points.push_back(c);
                }
                }
            }
            } break;
            default:
            // Handle other geometry types (MultiPolygon, MultiLineString, etc.) as needed
            break;
        }
        return geometry_msg;
    }

    GeoJson parseGeoJson(const std::string& file_path) {
        GDALAllRegister();
        GDALDatasetUniquePtr ds(static_cast<GDALDataset*>(
            GDALOpenEx(file_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)));
        if (!ds) {
            throw std::runtime_error("Could not open file with GDAL: " + file_path);
        }

        GeoJson geo_json_msg;
        geo_json_msg.num_features = 0;

        OGRLayer* layer = ds->GetLayer(0);
        if (!layer) {
            throw std::runtime_error("No layers found in data source.");
        }

        // Use exportToWkt() instead of ExportToWkt()
        if (layer->GetSpatialRef()) {
            char* pszWkt = nullptr;
            layer->GetSpatialRef()->exportToWkt(&pszWkt);
            if (pszWkt) {
                geo_json_msg.projection = pszWkt;
                CPLFree(pszWkt);
            }
        }

        layer->ResetReading();
        OGRFeature* poFeature;
        while ((poFeature = layer->GetNextFeature()) != nullptr) {
            Feature feature_msg;
            OGRGeometry* geom = poFeature->GetGeometryRef();
            if (geom) {
                feature_msg.geometry = convertOGRGeometry(geom);
            }
            feature_msg.properties = readProperties(poFeature);
            geo_json_msg.features.push_back(feature_msg);
            //loop each property
            for(auto prop : feature_msg.properties){
                if(prop.key == "uuid"){
                    feature_msg.uuid = prop.value;
                }
            }
            OGRFeature::DestroyFeature(poFeature);
        }
        geo_json_msg.num_features = geo_json_msg.features.size();

        return geo_json_msg;
    }
} // namespace geojson_parser
