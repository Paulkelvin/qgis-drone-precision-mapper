# -*- coding: utf-8 -*-
"""
Image processing module for Drone Precision Mapper
Contains all the georeferencing and coordinate transformation logic
"""

import os
import math
import numpy as np
import traceback
import tempfile
from pathlib import Path
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.Qt import Qt
import hashlib

# QGIS imports
from qgis.core import (
    QgsProject, QgsRasterLayer, QgsRaster, QgsPointXY,
    QgsCoordinateReferenceSystem
)
from qgis.gui import QgsMapToolEmitPoint, QgsVertexMarker
from qgis.utils import iface

# GDAL imports
from osgeo import gdal, osr

# PIL imports
from PIL import Image, ExifTags
from PIL.TiffImagePlugin import IFDRational

# Configure GDAL
gdal.UseExceptions()
gdal.AllRegister()


def convert_rational_to_float(value, log_func=None):
    """Converts an EXIF value to a float."""
    # Case 1: Pillow's special IFDRational type
    if isinstance(value, IFDRational):
        return float(value)
        
    # Case 2: Standard rational tuple (numerator, denominator)
    if isinstance(value, tuple) and len(value) == 2:
        try:
            num = float(value[0])
            den = float(value[1])
            return 0.0 if den == 0 else num / den
        except (ValueError, TypeError):
            pass

    # Case 3: Already a number (int or float)
    if isinstance(value, (int, float)):
        return float(value)

    # Case 4: A single number in a tuple/list, e.g., (5.0,)
    if isinstance(value, (tuple, list)) and len(value) == 1:
        try:
            return float(value[0])
        except (ValueError, TypeError):
            pass

    # Case 5: A number represented as a string, e.g., '5.0'
    if isinstance(value, str):
        try:
            return float(value)
        except (ValueError, TypeError):
            pass

    # If all conversions fail, print a warning and return 0.0
    if log_func:
        log_func(f"Warning: Could not convert value '{value}' to float. Using 0.0.")
    return 0.0
    
def convert_dms_to_degrees(value, log_func=None):
    """Converts a GPS coordinate from Degrees/Minutes/Seconds (DMS) format to decimal degrees."""
    d = convert_rational_to_float(value[0], log_func)
    m = convert_rational_to_float(value[1], log_func)
    s = convert_rational_to_float(value[2], log_func)
    return d + (m / 60.0) + (s / 3600.0)
    
def get_exif_data(img, log_func=None):
    """Extracts and decodes EXIF data from a Pillow Image object."""
    exif_data = {}
    try:
        info = img.getexif()
        if not info:
            # Don't log warning here - let the calling function handle it
            return {}

        for tag_id, value in info.items():
            tag_name = ExifTags.TAGS.get(tag_id, tag_id)
            if tag_name != "GPSInfo":
                exif_data[tag_name] = value

        try:
            gps_ifd = info.get_ifd(ExifTags.IFD.GPSInfo)
            if gps_ifd:
                gps_data = {}
                for tag_id, value in gps_ifd.items():
                    sub_tag_name = ExifTags.GPSTAGS.get(tag_id, tag_id)
                    gps_data[sub_tag_name] = value
                exif_data["GPSInfo"] = gps_data
        except (KeyError, TypeError):
            if log_func:
                log_func("Warning: GPSInfo tag found, but failed to retrieve GPS data dictionary (IFD).")
        except Exception as e:
            if log_func:
                log_func(f"Warning: An unexpected error occurred while processing GPS data: {e}")

        exif_data['LensModel'] = info.get(42036, 'Unknown')
        return exif_data
    except Exception as e:
        if log_func:
            log_func(f"❌ Error: Failed to parse EXIF metadata (corrupted or non-standard EXIF). This image cannot be processed. Details: {e}")
        return None
    
def get_lens_correction_from_exif(image_path, log_func=None):
    """Attempts to extract lens distortion parameters from EXIF."""
    try:
        import exiftool
        with exiftool.ExifTool() as et:
            metadata = et.get_metadata(image_path)
            distortion_params_str = metadata.get("XMP:LensDistortionParams", "0.0 0.0 0.0")
            k1 = float(distortion_params_str.split()[0])
            if log_func:
                log_func(f"Found lens distortion parameter k1: {k1}")
            return k1
    except ImportError:
        if log_func:
            log_func("ExifTool not available - using default lens correction value")
        return 0.002
    except Exception as e:
        if log_func:
            log_func(f"Lens correction extraction failed, using fallback value. Reason: {e}")
        return 0.002
        
def apply_distortion_correction(px, py, width, height, k1):
    """Applies a simple radial distortion correction (k1) to a pixel coordinate."""
    if k1 == 0:
        return px, py

    center_x, center_y = width / 2.0, height / 2.0
    xn = (px - center_x) / center_x
    yn = (py - center_y) / center_y
    r_sq = xn**2 + yn**2
    correction_factor = 1 + k1 * r_sq
    xn_corr = xn * correction_factor
    yn_corr = yn * correction_factor
    px_corr = center_x + xn_corr * center_x
    py_corr = center_y + yn_corr * center_y
    return px_corr, py_corr
        
def georeference_image(image_path, log_func=None):
    """Reads an image, extracts EXIF GPS data, and creates a georeferenced raster layer."""
    temp_path = None
    mem_ds = None
    driver = None
    layer = None
    
    # Check file size first (warn if > 100MB)
    file_size_mb = Path(image_path).stat().st_size / (1024 * 1024)
    if file_size_mb > 100:
        if log_func:
            log_func(f"⚠️ Warning: Large image file ({file_size_mb:.1f}MB) may take longer to process")
    
    # Always try to load as a QGIS raster first
    layer_name = Path(image_path).stem
    raster_layer = QgsRasterLayer(image_path, layer_name)
    if raster_layer.isValid() and raster_layer.crs().isValid() and raster_layer.extent().isFinite():
        QgsProject.instance().addMapLayer(raster_layer)
        if log_func:
            log_func("ℹ️ This image is already georeferenced (QGIS can load it). Advanced correction is disabled, but you can still pick coordinates.")
        return 'basic_picker'
    # If not a valid raster, try to open as a photo and extract EXIF
    try:
        with Image.open(image_path) as img:
            img = img.convert('RGB')
            
            # Check image dimensions (warn if > 50MP)
            width, height = img.size
            total_pixels = width * height
            if total_pixels > 50000000:  # 50MP limit
                if log_func:
                    log_func(f"⚠️ Warning: Large image ({width}x{height} = {total_pixels/1000000:.1f}MP) may take longer to process")
            
            exif = get_exif_data(img, log_func)  # Pass log_func to get clean error messages
            if not exif:
                if log_func:
                    log_func("❌ Error: This image does not contain EXIF metadata and cannot be processed.")
                return None
        gps_info = exif.get("GPSInfo")
        if not gps_info or 'GPSLatitude' not in gps_info or 'GPSLongitude' not in gps_info:
            if log_func:
                log_func("❌ Error: This image does not contain GPS EXIF data and cannot be processed.")
            return None

        # Extract GPS coordinates
        lat = convert_dms_to_degrees(gps_info['GPSLatitude'], log_func)
        if gps_info.get('GPSLatitudeRef') == 'S': lat = -lat
        lon = convert_dms_to_degrees(gps_info['GPSLongitude'], log_func)
        if gps_info.get('GPSLongitudeRef') == 'W': lon = -lon

        # Use altitude from EXIF or fallback to default value
        altitude = convert_rational_to_float(gps_info.get('GPSAltitude', 100.0), log_func)
        if altitude == 0:
            if log_func:
                log_func("❌ Error: Altitude in EXIF metadata is zero. This image cannot be processed.")
            return None
        
        # Validate altitude is within reasonable range (1m to 10km)
        if altitude < 1.0 or altitude > 10000.0:
            if log_func:
                log_func(f"❌ Error: Altitude value ({altitude:.1f}m) is outside reasonable range (1-10,000m). This image cannot be processed.")
            return None
        
        # Log if using fallback altitude
        if 'GPSAltitude' not in gps_info and log_func:
            log_func("ℹ️ Using default altitude (100.0m) - GPSAltitude not found in EXIF")
        
        # Use focal length from EXIF or fallback to default value
        focal_length = convert_rational_to_float(exif.get('FocalLength', 8.0), log_func)
        if focal_length == 0:
            if log_func:
                log_func("❌ Error: FocalLength in EXIF metadata is zero. This image cannot be processed.")
            return None
        
        # Validate focal length is within reasonable range (1mm to 500mm)
        if focal_length < 1.0 or focal_length > 500.0:
            if log_func:
                log_func(f"❌ Error: Focal length value ({focal_length:.1f}mm) is outside reasonable range (1-500mm). This image cannot be processed.")
            return None
        
        # Log if using fallback focal length
        if 'FocalLength' not in exif and log_func:
            log_func("ℹ️ Using default focal length (8.0mm) - FocalLength not found in EXIF")
        
        width, height = img.size
        
        # Validate sensor width (should be reasonable for drone cameras)
        sensor_width_mm = 6.17  # Default for many drone cameras
        if 'FocalPlaneXResolution' in exif and 'FocalPlaneYResolution' in exif:
            try:
                x_res = convert_rational_to_float(exif['FocalPlaneXResolution'], log_func)
                y_res = convert_rational_to_float(exif['FocalPlaneYResolution'], log_func)
                if x_res > 0 and y_res > 0:
                    # Calculate sensor width from resolution
                    sensor_width_mm = width / x_res * 25.4  # Convert to mm
                    if sensor_width_mm < 1.0 or sensor_width_mm > 50.0:
                        if log_func:
                            log_func(f"⚠️ Warning: Calculated sensor width ({sensor_width_mm:.2f}mm) seems unusual, using default (6.17mm)")
                        sensor_width_mm = 6.17
                    elif log_func:
                        log_func(f"ℹ️ Using calculated sensor width: {sensor_width_mm:.2f}mm")
            except:
                if log_func:
                    log_func("ℹ️ Using default sensor width (6.17mm) - could not calculate from EXIF")
        else:
            if log_func:
                log_func("ℹ️ Using default sensor width (6.17mm) - sensor parameters not found in EXIF")
        
        ground_width_m = (sensor_width_mm * altitude) / focal_length
        meters_per_pixel = ground_width_m / width

        # Validate ground resolution is within reasonable range (0.1cm to 10m per pixel)
        if meters_per_pixel < 0.001 or meters_per_pixel > 10.0:
            if log_func:
                log_func(f"❌ Error: Calculated ground resolution ({meters_per_pixel:.3f}m/pixel) is outside reasonable range (0.001-10m/pixel). This suggests invalid camera parameters.")
            return None

        m_per_deg_lon = 111320 * math.cos(math.radians(lat))
        m_per_deg_lat = 111320
        pixel_width_deg = meters_per_pixel / m_per_deg_lon
        pixel_height_deg = meters_per_pixel / m_per_deg_lat
        top_left_lon = lon - (width / 2.0) * pixel_width_deg
        top_left_lat = lat + (height / 2.0) * pixel_height_deg
        geotransform = (top_left_lon, pixel_width_deg, 0, top_left_lat, 0, -pixel_height_deg)
        mem_ds = gdal.GetDriverByName('MEM').Create('', width, height, 3, gdal.GDT_Byte)
        mem_ds.SetGeoTransform(geotransform)
        srs = osr.SpatialReference()
        srs.ImportFromEPSG(4326)
        mem_ds.SetProjection(srs.ExportToWkt())
        np_img = np.array(img)
        for i in range(3):
            mem_ds.GetRasterBand(i + 1).WriteArray(np_img[:, :, i])
        with tempfile.NamedTemporaryFile(suffix='.tif', delete=False) as tmp_file:
            temp_path = tmp_file.name
        driver = gdal.GetDriverByName('GTiff')
        try:
            output_ds = driver.CreateCopy(temp_path, mem_ds)
            if output_ds is None:
                if log_func:
                    log_func(f"❌ Error: GDAL CreateCopy failed for path: {temp_path}")
                return None
            output_ds = None
        except Exception as e:
            if log_func:
                log_func(f"❌ Error: GDAL CreateCopy error: {e}")
            return None
        mem_ds = None 
        driver = None
        layer_name = Path(image_path).stem
        layer = QgsRasterLayer(temp_path, layer_name)
        if not layer.isValid():
            if log_func:
                log_func(f"❌ Error: Failed to load the georeferenced layer: {layer.error().summary()}")
            return None
        QgsProject.instance().addMapLayer(layer)
        distortion_k1 = get_lens_correction_from_exif(image_path, log_func)
        if log_func:
            log_func(f"Center Coordinates: Lat={lat:.6f}°, Lon={lon:.6f}°")
            log_func(f"Altitude: {altitude:.2f} m | Ground Resolution: {meters_per_pixel:.3f} m/pixel")
        # Set custom properties for robust identification
        layer.setCustomProperty("DronePrecisionMapper:source_image", str(image_path))
        try:
            with open(image_path, "rb") as f:
                file_hash = hashlib.md5(f.read()).hexdigest()
            layer.setCustomProperty("DronePrecisionMapper:file_hash", file_hash)
        except Exception as e:
            file_hash = None
            if log_func:
                log_func(f"⚠️ Could not compute file hash: {e}")
        return {
            'image_width': width, 'image_height': height,
            'distortion_k1': distortion_k1, 'layer': layer,
            'layer_path': image_path,  # for reference
            'layer_source': layer.source(),
            'file_hash': file_hash,
            'image_path': image_path
        }
    except Exception as e:
        if log_func:
            log_func(f"❌ Error: This file cannot be loaded as a raster in QGIS and is not a supported drone photo. Please check the file format. (Details: {e})")
        return None


def process_multiple_drone_photos(folder_path, log_func=None, progress_bar=None):
    """Process multiple drone photos in a folder for batch georeferencing."""
    image_extensions = ['.jpg', '.jpeg', '.tiff', '.tif', '.png']
    image_folder = Path(folder_path)
    
    if not image_folder.exists():
        if log_func:
            log_func(f"Error: Folder does not exist: {folder_path}")
        return []
    
    # Find all image files
    image_files = []
    for ext in image_extensions:
        image_files.extend(image_folder.glob(f"*{ext}"))
        image_files.extend(image_folder.glob(f"*{ext.upper()}"))
    
    if not image_files:
        if log_func:
            log_func(f"No image files found in: {folder_path}")
        return []
    
    # Check batch size limits
    if len(image_files) > 100:
        if log_func:
            log_func(f"⚠️ Warning: Large batch ({len(image_files)} images) may take a long time to process")
            log_func("💡 Tip: Consider processing in smaller batches for better performance")
    
    if len(image_files) > 500:
        if log_func:
            log_func(f"❌ Error: Too many images ({len(image_files)}) for batch processing. Maximum is 500 images.")
        return []
    
    if log_func:
        log_func(f"Found {len(image_files)} image files to process...")
    
    processed_layers = []
    successful_count = 0
    failed_count = 0
    
    for i, image_path in enumerate(image_files):
        if log_func:
            if i > 0:
                log_func("="*50 + "\n")
            log_func(f"[{i+1}/{len(image_files)}] Processing: {image_path.name}")
            log_func("-"*50)
        if progress_bar:
            progress_bar.setValue(int((i / len(image_files)) * 100))
            progress_bar.setVisible(True)
            # Allow user to cancel by checking if progress bar was reset
            if progress_bar.value() == 0 and i > 0:
                if log_func:
                    log_func("⏹️ Batch processing cancelled by user")
                break
        try:
            # Extract EXIF and image details for reporting
            img = Image.open(image_path)
            exif = get_exif_data(img)
            width, height = img.size
            camera_model = exif.get('Model', 'Unknown')
            lens_model = exif.get('LensModel', 'Unknown')
            date_time = exif.get('DateTime', 'Unknown')
            gps_info = exif.get('GPSInfo', {})
            gps_status = 'Yes' if ('GPSLatitude' in gps_info and 'GPSLongitude' in gps_info) else 'No'
            focal_length = exif.get('FocalLength', None)
            sensor_width = 6.17  # Default
            if 'FocalLength' not in exif:
                focal_length_str = '8.0 (default, not found in EXIF)'
            else:
                focal_length_str = f"{convert_rational_to_float(focal_length)}"
            if 'Model' in exif and 'DJI' in str(exif['Model']):
                sensor_width = 6.17  # DJI default
            # Log extracted details
            if log_func:
                log_func(f"Image size: {width} x {height} px")
                log_func(f"Camera: {camera_model}")
                log_func(f"Lens: {lens_model}")
                log_func(f"Captured: {date_time}")
                log_func(f"GPS data found in EXIF: {gps_status}")
                log_func(f"Focal Length: {focal_length_str} mm | Sensor Width: {sensor_width} mm")
            result_params = georeference_image(str(image_path), log_func)
            if result_params == 'basic_picker':
                layer_name = Path(image_path).stem
                layer = QgsProject.instance().mapLayersByName(layer_name)[-1]
                processed_layers.append({
                    'image_path': str(image_path),
                    'params': {
                        'layer': layer,
                        'image_width': layer.width(),
                        'image_height': layer.height(),
                        'distortion_k1': 0.0,
                        'layer_path': str(image_path),
                        'layer_id': layer.id(),
                        'original_extent': layer.extent()
                    }
                })
                successful_count += 1
                if log_func:
                    log_func(f"QGIS Layer: {layer.name()}")
                    log_func(f"✅ {image_path.name}")
            elif result_params:
                processed_layers.append({
                    'image_path': str(image_path),
                    'params': result_params
                })
                successful_count += 1
                # Lens distortion
                k1 = result_params.get('distortion_k1', None)
                if log_func and k1 is not None:
                    log_func(f"Lens Distortion (k1): {k1}")
                # QGIS layer name
                layer = result_params.get('layer', None)
                if log_func and layer is not None:
                    log_func(f"QGIS Layer: {layer.name()}")
                # Print summary info for the image if available
                lat = result_params.get('lat', None)
                lon = result_params.get('lon', None)
                altitude = result_params.get('altitude', None)
                meters_per_pixel = result_params.get('meters_per_pixel', None)
                if log_func and lat is not None and lon is not None:
                    log_func(f"Center Coordinates: Lat={lat:.6f}°, Lon={lon:.6f}°")
                if log_func and altitude is not None and meters_per_pixel is not None:
                    log_func(f"Altitude: {altitude:.2f} m | Ground Resolution: {meters_per_pixel:.3f} m/pixel")
                if log_func:
                    log_func(f"✅ {image_path.name}")
            else:
                failed_count += 1
                if log_func:
                    log_func(f"❌ {image_path.name} (Skipped: missing or invalid EXIF, GPS, altitude, or focal length)")
        except Exception as e:
            failed_count += 1
            if log_func:
                log_func(f"❌ {image_path.name}: {str(e)} (Skipped: missing or invalid EXIF, GPS, altitude, or focal length)")
    
    if progress_bar:
        progress_bar.setValue(100)
    
    if log_func:
        log_func(f"Batch processing complete: {successful_count} successful, {failed_count} failed out of {len(image_files)} total")
    
    return processed_layers


# Utility to load/save persistent counters

def load_persistent_counters():
    try:
        project = QgsProject.instance()
        counters_str = project.readEntry("DronePrecisionMapper", "click_counters", "{}")
        import json
        return json.loads(counters_str[0])
    except Exception as e:
        print(f"[DEBUG] Failed to load persistent counters: {e}")
        return {}

def save_persistent_counters(counters):
    try:
        project = QgsProject.instance()
        import json
        counters_str = json.dumps(counters)
        project.writeEntry("DronePrecisionMapper", "click_counters", counters_str)
    except Exception as e:
        print(f"[DEBUG] Failed to save persistent counters: {e}")


class PrecisionClickTool(QgsMapToolEmitPoint):
    """A QGIS Map Tool that corrects a clicked point for lens distortion."""
    
    def __init__(self, canvas, georef_params, coord_callback=None, persistent_counters=None):
        super().__init__(canvas)
        self.georef_params = georef_params
        self.canvas = canvas
        self.coord_callback = coord_callback
        self.persistent_counters = persistent_counters or {}  # Use persistent counters from plugin
        self.image_path = georef_params.get('image_path', '')  # Store image path for stable counter key
        
        # Create more visible markers
        self.marker = QgsVertexMarker(canvas)
        self.marker.setColor(QColor("red"))
        self.marker.setIconSize(15)  # Increased size
        self.marker.setIconType(QgsVertexMarker.ICON_CROSS)
        self.marker.setPenWidth(3)  # Increased pen width
        self.marker.setVisible(False)  # Start hidden
        
        self.uncorrected_marker = QgsVertexMarker(canvas)
        self.uncorrected_marker.setColor(QColor("cyan"))
        self.uncorrected_marker.setIconSize(12)  # Increased size
        self.uncorrected_marker.setIconType(QgsVertexMarker.ICON_BOX)
        self.uncorrected_marker.setPenWidth(2)  # Increased pen width
        self.uncorrected_marker.setVisible(False)  # Start hidden
        
        # Set cursor to indicate this is an active tool
        self.setCursor(Qt.CrossCursor)
        
        print("PrecisionClickTool initialized successfully")
        print(f"Layer: {georef_params.get('layer', 'None')}")
        print(f"Image path: {self.image_path}")
        print(f"Image dimensions: {georef_params.get('image_width', 'Unknown')}x{georef_params.get('image_height', 'Unknown')}")

    def canvasReleaseEvent(self, event):
        print("Canvas release event triggered!")
        # Get the active layer at click time
        active_layer = iface.activeLayer()
        if not active_layer:
            if self.coord_callback:
                self.coord_callback("❌ No active layer.")
            return
        # Get the persistent identifier
        file_hash = active_layer.customProperty("DronePrecisionMapper:file_hash", None)
        source_image = active_layer.customProperty("DronePrecisionMapper:source_image", None)
        if not file_hash and not source_image:
            if self.coord_callback:
                self.coord_callback("❌ The selected layer is not a processed drone image.")
            return
        # Use file_hash as the counter key if available, else source_image
        counter_key = file_hash or source_image
        # Increment the correct counter
        self.persistent_counters = load_persistent_counters()
        click_count = self.persistent_counters.get(counter_key, 0) + 1
        self.persistent_counters[counter_key] = click_count
        save_persistent_counters(self.persistent_counters)
        # Use the original image name for display
        image_name = Path(source_image).stem if source_image else "Unknown"
        clicked_map_point = self.toMapCoordinates(event.pos())
        # Show uncorrected marker immediately
        self.uncorrected_marker.setCenter(clicked_map_point)
        self.uncorrected_marker.setVisible(True)
        # Get coordinate transformation
        provider = active_layer.dataProvider()
        geotransform = provider.extent()
        if not geotransform.isNull():
            width = provider.xSize()
            height = provider.ySize()
            try:
                pixel_width = geotransform.width() / width
                pixel_height = geotransform.height() / height
                x_origin = geotransform.xMinimum()
                y_origin = geotransform.yMaximum()
                px = (clicked_map_point.x() - x_origin) / pixel_width
                py = (y_origin - clicked_map_point.y()) / pixel_height
                pixel_tolerance = 5
                if px < -pixel_tolerance or px >= width + pixel_tolerance or py < -pixel_tolerance or py >= height + pixel_tolerance:
                    if self.coord_callback:
                        self.coord_callback("⚠️ Click outside image pixel bounds")
                    return
                px = max(0, min(width - 1, px))
                py = max(0, min(height - 1, py))
                px_corr, py_corr = apply_distortion_correction(
                    px, py,
                    self.georef_params['image_width'], self.georef_params['image_height'],
                    self.georef_params['distortion_k1'])
                corrected_map_point = QgsPointXY(
                    x_origin + px_corr * pixel_width,
                    y_origin - py_corr * pixel_height
                )
                dist_m = clicked_map_point.distance(corrected_map_point) * 111320
                coord_message = f"📍 {image_name} - Click {click_count}:\n"
                coord_message += f"   Uncorrected: Lat={clicked_map_point.y():.6f}, Lon={clicked_map_point.x():.6f}\n"
                coord_message += f"   Corrected:   Lat={corrected_map_point.y():.6f}, Lon={corrected_map_point.x():.6f}\n"
                coord_message += f"   Correction:  ~{dist_m:.3f} meters\n"
                if self.coord_callback:
                    self.coord_callback(coord_message)
                self.marker.setCenter(corrected_map_point)
                self.marker.setVisible(True)
                self.canvas.refresh()
            except Exception as e:
                print(f"Error in coordinate transformation: {e}")
                if self.coord_callback:
                    self.coord_callback(f"❌ Error in coordinate calculation: {e}")
                return
        else:
            print("Error: Could not get geotransform from layer")

    def deactivate(self):
        """Clean up when tool is deactivated."""
        if self.marker:
            self.marker.setVisible(False)
            self.marker.setCenter(QgsPointXY(0, 0))  # Reset position
        if self.uncorrected_marker:
            self.uncorrected_marker.setVisible(False)
            self.uncorrected_marker.setCenter(QgsPointXY(0, 0))  # Reset position
        super().deactivate()


class MultiLayerPrecisionClickTool(QgsMapToolEmitPoint):
    """A QGIS Map Tool that works with multiple layers, always using the QGIS active layer."""
    
    def __init__(self, canvas, processed_layers, coord_callback=None, persistent_counters=None):
        super().__init__(canvas)
        self.processed_layers = processed_layers
        self.canvas = canvas
        self.coord_callback = coord_callback
        self.persistent_counters = persistent_counters or {}
        self.markers = []
        self.uncorrected_markers = []
        for i, layer_data in enumerate(processed_layers):
            marker = QgsVertexMarker(canvas)
            marker.setColor(QColor("red"))
            marker.setIconSize(15)
            marker.setIconType(QgsVertexMarker.ICON_CROSS)
            marker.setPenWidth(3)
            marker.setVisible(False)
            uncorrected_marker = QgsVertexMarker(canvas)
            uncorrected_marker.setColor(QColor("cyan"))
            uncorrected_marker.setIconSize(12)
            uncorrected_marker.setIconType(QgsVertexMarker.ICON_BOX)
            uncorrected_marker.setPenWidth(2)
            uncorrected_marker.setVisible(False)
            self.markers.append(marker)
            self.uncorrected_markers.append(uncorrected_marker)
        self.setCursor(Qt.CrossCursor)
        print(f"\n--- Multi-Layer Precision Tool Activated ---")
        print(f"Use the QGIS Layers Panel to select the active image layer.")
        print(f"Click on the image to get distortion-corrected coordinates.")

    def canvasReleaseEvent(self, event):
        active_layer = iface.activeLayer()
        if not active_layer:
            self.coord_callback("❌ No active layer.")
            return
        file_hash = active_layer.customProperty("DronePrecisionMapper:file_hash", None)
        source_image = active_layer.customProperty("DronePrecisionMapper:source_image", None)
        if not file_hash and not source_image:
            self.coord_callback("❌ The selected layer is not a processed drone image.")
            return
        counter_key = file_hash or source_image
        self.persistent_counters = load_persistent_counters()
        click_count = self.persistent_counters.get(counter_key, 0) + 1
        self.persistent_counters[counter_key] = click_count
        save_persistent_counters(self.persistent_counters)
        image_name = Path(source_image).stem if source_image else "Unknown"
        clicked_map_point = self.toMapCoordinates(event.pos())
        # Show uncorrected marker for this layer (find marker by active layer)
        marker_idx = None
        for i, layer_data in enumerate(self.processed_layers):
            proc_layer = layer_data['params']['layer']
            proc_hash = proc_layer.customProperty("DronePrecisionMapper:file_hash", None)
            proc_source = proc_layer.customProperty("DronePrecisionMapper:source_image", None)
            if (proc_hash and file_hash and proc_hash == file_hash) or (proc_source and source_image and proc_source == source_image):
                marker_idx = i
                break
        if marker_idx is None:
            self.coord_callback("❌ Could not match the active layer to a processed drone image.")
            return
        for m in self.markers: m.setVisible(False)
        for m in self.uncorrected_markers: m.setVisible(False)
        self.uncorrected_markers[marker_idx].setCenter(clicked_map_point)
        self.uncorrected_markers[marker_idx].setVisible(True)
        provider = active_layer.dataProvider()
        geotransform = provider.extent()
        if not geotransform.isNull():
            width = provider.xSize()
            height = provider.ySize()
            try:
                pixel_width = geotransform.width() / width
                pixel_height = geotransform.height() / height
                x_origin = geotransform.xMinimum()
                y_origin = geotransform.yMaximum()
                px = (clicked_map_point.x() - x_origin) / pixel_width
                py = (y_origin - clicked_map_point.y()) / pixel_height
                pixel_tolerance = 5
                if px < -pixel_tolerance or px >= width + pixel_tolerance or py < -pixel_tolerance or py >= height + pixel_tolerance:
                    self.coord_callback("⚠️ Click outside image pixel bounds")
                    return
                px = max(0, min(width - 1, px))
                py = max(0, min(height - 1, py))
                px_corr, py_corr = apply_distortion_correction(
                    px, py,
                    self.processed_layers[marker_idx]['params']['image_width'],
                    self.processed_layers[marker_idx]['params']['image_height'],
                    self.processed_layers[marker_idx]['params']['distortion_k1'])
                corrected_map_point = QgsPointXY(
                    x_origin + px_corr * pixel_width,
                    y_origin - py_corr * pixel_height
                )
                dist_m = clicked_map_point.distance(corrected_map_point) * 111320
                coord_message = f"📍 {image_name} - Click {click_count}:\n"
                coord_message += f"   Uncorrected: Lat={clicked_map_point.y():.6f}, Lon={clicked_map_point.x():.6f}\n"
                coord_message += f"   Corrected:   Lat={corrected_map_point.y():.6f}, Lon={corrected_map_point.x():.6f}\n"
                coord_message += f"   Correction:  ~{dist_m:.3f} meters\n"
                self.coord_callback(coord_message)
                self.markers[marker_idx].setCenter(corrected_map_point)
                self.markers[marker_idx].setVisible(True)
                self.canvas.refresh()
            except Exception as e:
                print(f"Error in coordinate transformation: {e}")
                self.coord_callback(f"❌ Error in coordinate calculation: {e}")
                return
        else:
            print("Error: Could not get geotransform from layer")


class BasicCoordinatePickerTool(QgsMapToolEmitPoint):
    """A QGIS Map Tool for picking coordinates on already-georeferenced rasters (no EXIF, no correction)."""
    def __init__(self, canvas, layer, coord_callback=None):
        super().__init__(canvas)
        self.canvas = canvas
        self.layer = layer
        self.coord_callback = coord_callback
        self.click_counter = 0  # Add click counter
        self.marker = QgsVertexMarker(canvas)
        self.marker.setColor(QColor("blue"))
        self.marker.setIconSize(12)
        self.marker.setIconType(QgsVertexMarker.ICON_CIRCLE)
        self.marker.setPenWidth(2)
        self.marker.setVisible(False)
        self.setCursor(Qt.CrossCursor)

    def canvasReleaseEvent(self, event):
        print("Basic coordinate picker canvas release event triggered!")
        
        # Try to get the current layer more robustly
        layer = self.layer
        
        # If the stored layer is no longer valid, try to find it by name
        if not layer or not layer.isValid():
            # Try to find the layer by name in the project
            all_layers = QgsProject.instance().mapLayers().values()
            raster_layers = [l for l in all_layers if isinstance(l, QgsRasterLayer)]
            
            if raster_layers:
                # Use the first raster layer as fallback
                layer = raster_layers[0]
                print(f"Using fallback layer: {layer.name()}")
        
        if not layer or not isinstance(layer, QgsRasterLayer):
            print("Error: Invalid layer")
            if self.coord_callback:
                self.coord_callback("❌ Error: Could not find the image layer. Please reload the image.")
            return

        clicked_map_point = self.toMapCoordinates(event.pos())
        print(f"Clicked at map coordinates: {clicked_map_point.x():.6f}, {clicked_map_point.y():.6f}")
        print(f"Layer extent: {layer.extent().toString()}")
        
        # Check if click is within the layer extent with more tolerance
        layer_extent = layer.extent()
        # Increase tolerance significantly to account for coordinate precision issues and edge cases
        tolerance = 0.00001  # About 10 meters in degrees (increased from 1 meter)
        
        # More flexible bounds checking - check if point is reasonably close to the layer
        x_min, x_max = layer_extent.xMinimum() - tolerance, layer_extent.xMaximum() + tolerance
        y_min, y_max = layer_extent.yMinimum() - tolerance, layer_extent.yMaximum() + tolerance
        
        if not (x_min <= clicked_map_point.x() <= x_max and y_min <= clicked_map_point.y() <= y_max):
            # Additional check: if the point is very close to the bounds, allow it anyway
            distance_to_bounds = min(
                abs(clicked_map_point.x() - x_min), abs(clicked_map_point.x() - x_max),
                abs(clicked_map_point.y() - y_min), abs(clicked_map_point.y() - y_max)
            )
            
            # If within 5 meters of bounds, allow the click
            if distance_to_bounds > 0.00005:  # About 5 meters in degrees
                if self.coord_callback:
                    self.coord_callback(f"⚠️ Click outside image bounds - please click within the {layer.name()} area")
                return
            else:
                print(f"Click near bounds (distance: {distance_to_bounds:.6f}), allowing it")
        
        # Increment click counter
        self.click_counter += 1
        
        # Show marker
        self.marker.setCenter(clicked_map_point)
        self.marker.setVisible(True)
        
        # Display coordinates
        if self.coord_callback:
            self.coord_callback(f"📍 Coordinates (Click {self.click_counter}): Lat={clicked_map_point.y():.6f}°, Lon={clicked_map_point.x():.6f}°")

    def deactivate(self):
        if self.marker:
            self.marker.setVisible(False)
            self.marker.setCenter(QgsPointXY(0, 0))  # Reset position
        super().deactivate() 