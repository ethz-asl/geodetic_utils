New library for geodetic data. I name it geotf.
For reference: From now on, geoframes are coordinate frames handled by geotf. TF frames are handled by regular tf.

Core features:
- Supports all frames supported by GDAL (https://gdal.org/) which is pretty much everything in existance.
- Can seamlessly and transparently convert between multiple geoframes 
- Supports mapping between geo frames and tf frames (through a common frame)
- Has methods to publish geodetic locations as tf frames
- Configuration of frames through rosparam (as they are static).

Example querys that are possible:
- GPS WGS 84 position in ENU frame
- UTM coordinates in ENU frame
- Current camera position in GPS coordinates (through camera_tf -> enu_tf = enu_geo -> gps_geo)
- Any UTM coordinate in current end-effector frame
- Any GPS coordinate in current camera frame.
- and so on.

Needed configurations:
- See demo.launch for an example
- Basically you configure geoframes with their names and their definition.
- Definition can be of type:
  - ENUOrigin (lon, lat, alt)
  - Well known geocode (GCS), such as "WGS84"
  - UTM zone (e.g. 32 north)
  - EPSG code. see https://epsg.io. Well known examples: New swiss coordinates (CH1903+, EPSG code 2056, see https://epsg.io/2056). Less known example: Maupiti 83 frame for some island in the pacific, EPSG code 4692, https://epsg.io/4692. 
- If TF adapter are used, a specification of which geo frame equals which tf frame. This is usually the ENU frame in some sort.

Especially through the EPSG facility, we probably never need to implement any other geodetic coordinate frame ever again, as basically every frame is defined in that database.


Publishing TF frames:
- Can publish any geolocation as TF frame based on the geo frame only. See demo_node.

