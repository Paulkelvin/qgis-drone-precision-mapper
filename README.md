# Drone Precision Mapper for QGIS

A QGIS plugin for fast, accurate georeferencing and coordinate extraction from drone images, with automatic lens distortion correction and batch processing support.

## Features
- **Automatic Georeferencing:** Reads GPS data from drone image EXIF and places images on the map.
- **Lens Distortion Correction:** Applies correction using camera metadata for precise mapping.
- **Precision Coordinate Picking:** Click on images to get distortion-corrected, real-world coordinates.
- **Batch Processing:** Georeference and load entire folders of drone images at once.
- **Seamless QGIS Integration:** Works directly in QGIS with a user-friendly interface.

## Requirements
- QGIS 3.x (with Python 3)
- Python packages: Pillow, numpy, PyQt5 (usually included with QGIS), GDAL
- (Optional) ExifTool for advanced lens correction

> **Note:** QGIS includes its own Python environment. You do **not** need to install Python separately for this plugin. All required Python packages should be available in a standard QGIS 3.x installation. If you encounter missing packages, use the OSGeo4W Shell (on Windows) or your system's package manager to install them within the QGIS environment.

## Installation
1. **Clone or download this repository.**
2. **Open a terminal and navigate to the directory containing `install_plugin.py`.**
3. **Install the plugin:**
   - Run the provided script:
     ```
     python install_plugin.py
     ```
   - This copies the plugin to your QGIS plugins directory.
4. **Start or Restart QGIS.**
5. **Enable the plugin:**
   - Go to `Plugins → Manage and Install Plugins` in QGIS.
   - Find and enable **Drone Precision Mapper**.

## Usage
1. **Open QGIS.**
2. **Access the plugin:**
   - Click on `Plugins` in the top menu.
   - Select `Manage and Install Plugins`.
   - Go to the `Installed` tab.
   - Find and enable **Drone Precision Mapper**.
   - The plugin will appear in the Plugins menu.
3. **Choose your mode:**
   - **Single Image:** Select a drone image to georeference and analyze.
   - **Batch Processing:** Select a folder to process multiple images at once.
4. **Click on loaded images to extract distortion-corrected coordinates.**
5. **Coordinate results appear in the plugin's dock widget and QGIS Python Console.**

After enabling, look for the Drone Precision Mapper icon in the QGIS toolbar:

![Drone Precision Mapper Icon](DronePrecisionMapper/icon.png)

---

![Screenshot: Drone Precision Mapper in QGIS](qgis_plugin_screenshot.png)
*Screenshot: Drone Precision Mapper in QGIS after processing single and batch images. The plugin window and coordinate picking results are shown.*

## Standalone Script
- The `dronemapping`