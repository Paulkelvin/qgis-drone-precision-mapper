# -*- coding: utf-8 -*-
"""
Drone Precision Mapper Plugin
Main plugin class for QGIS integration
"""

import os
import math
import numpy as np
import traceback
import tempfile
from pathlib import Path
from PyQt5.QtWidgets import (
    QAction, QFileDialog, QMessageBox, QProgressBar, 
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QGroupBox, QRadioButton, QLineEdit, QTextEdit,
    QWidget, QDialog, QApplication, QDockWidget
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QIcon, QColor

# QGIS imports
from qgis.core import (
    QgsProject, QgsRasterLayer, QgsRaster, QgsPointXY,
    QgsCoordinateReferenceSystem
)
from qgis.gui import QgsMapToolEmitPoint, QgsVertexMarker
from qgis.utils import iface

# Import our image processor
from .image_processor import georeference_image, process_multiple_drone_photos, PrecisionClickTool, MultiLayerPrecisionClickTool, BasicCoordinatePickerTool
import hashlib


class CoordinateDisplayDock(QDockWidget):
    """Persistent dock widget for displaying coordinates."""
    
    def __init__(self, parent=None):
        super().__init__("Coordinate Display", parent)
        self.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        
        # Create the widget content
        self.widget = QWidget()
        self.layout = QVBoxLayout(self.widget)
        
        # Title
        title_label = QLabel("📍 Click Coordinates")
        title_label.setStyleSheet("font-weight: bold; font-size: 14px; margin: 5px;")
        self.layout.addWidget(title_label)
        
        # Coordinate display
        self.coord_display = QTextEdit()
        self.coord_display.setMaximumHeight(300)
        self.coord_display.setReadOnly(True)
        self.coord_display.setPlaceholderText("Click on the image to see coordinates here...")
        self.coord_display.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                border: 1px solid #dee2e6;
                border-radius: 4px;
                padding: 8px;
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }
        """)
        self.layout.addWidget(self.coord_display)
        
        # Clear button
        clear_btn = QPushButton("Clear Display")
        clear_btn.clicked.connect(self.clear_display)
        self.layout.addWidget(clear_btn)
        
        self.setWidget(self.widget)
        
    def update_coordinates(self, message):
        """Update the coordinate display with new coordinates."""
        self.coord_display.append(message)
        # Auto-scroll to bottom
        cursor = self.coord_display.textCursor()
        cursor.movePosition(cursor.End)
        self.coord_display.setTextCursor(cursor)
        
    def clear_display(self):
        """Clear the coordinate display."""
        self.coord_display.clear()


class DronePrecisionMapper:
    """Main plugin class for Drone Precision Mapper."""
    
    def __init__(self, iface):
        """Initialize the plugin."""
        self.iface = iface
        self.canvas = iface.mapCanvas()
        self.dock_widget = None
        self.active_tool = None
        self.processed_layers = []
        self.coordinate_dock = None
        self.persistent_click_counters = self._load_persistent_counters()  # Load from QGIS project
        
        # Plugin actions
        self.action = None
        self.menu = None
        
    def _load_persistent_counters(self):
        """Load persistent click counters from QGIS project metadata."""
        try:
            project = QgsProject.instance()
            counters_str = project.readEntry("DronePrecisionMapper", "click_counters", "{}")[0]
            import json
            return json.loads(counters_str)
        except:
            return {}
    
    def _save_persistent_counters(self):
        """Save persistent click counters to QGIS project metadata."""
        try:
            project = QgsProject.instance()
            import json
            counters_str = json.dumps(self.persistent_click_counters)
            project.writeEntry("DronePrecisionMapper", "click_counters", counters_str)
        except Exception as e:
            print(f"Warning: Could not save persistent counters: {e}")
    
    def update_click_counter(self, image_path, new_count):
        """Update click counter and save to persistent storage."""
        self.persistent_click_counters[image_path] = new_count
        self._save_persistent_counters()
        
    def initGui(self):
        """Create the menu entries and toolbar icons inside the QGIS GUI."""
        # Create action
        icon_path = os.path.join(os.path.dirname(__file__), "icon.png")
        self.action = QAction(
            QIcon(icon_path),
            "Drone Precision Mapper",
            self.iface.mainWindow()
        )
        self.action.triggered.connect(self.run)
        self.action.setEnabled(True)
        
        # Add toolbar button and menu item
        self.iface.addToolBarIcon(self.action)
        self.iface.addPluginToMenu("&Drone Precision Mapper", self.action)
        
        # Create persistent coordinate display dock with error handling
        try:
            self.coordinate_dock = CoordinateDisplayDock(self.iface.mainWindow())
            self.iface.mainWindow().addDockWidget(Qt.RightDockWidgetArea, self.coordinate_dock)
            # Don't hide it initially - let it be visible
            self.coordinate_dock.show()
        except Exception as e:
            print(f"Warning: Could not create coordinate dock widget: {e}")
            self.coordinate_dock = None
        
    def unload(self):
        """Removes the plugin menu item and icon from QGIS GUI."""
        self.iface.removePluginMenu("&Drone Precision Mapper", self.action)
        self.iface.removeToolBarIcon(self.action)
        
        # Clean up any active tools
        if self.active_tool:
            self.canvas.unsetMapTool(self.active_tool)
            self.active_tool = None
            
        # Close dock widget if open
        if self.dock_widget:
            self.dock_widget.close()
            
        # Remove coordinate dock
        if self.coordinate_dock:
            self.coordinate_dock.close()
            
    def run(self):
        """Run method that performs all the real work."""
        try:
            # Ensure coordinate dock is visible
            if self.coordinate_dock:
                self.coordinate_dock.show()
                self.coordinate_dock.raise_()  # Bring to front
            else:
                print("Warning: Coordinate dock widget not available")
                
            # Create and show the main dialog
            self.dialog = DronePrecisionMapperDialog(self.iface, self)
            self.dialog.show()
            
        except Exception as e:
            print(f"Error opening Drone Precision Mapper dialog: {e}")
            # Show a simple error message to the user
            QMessageBox.critical(self.iface.mainWindow(), "Error", 
                               f"Failed to open Drone Precision Mapper: {str(e)}")


class DronePrecisionMapperDialog(QDialog):
    """Main dialog for the Drone Precision Mapper plugin."""
    
    def __init__(self, iface, plugin):
        super().__init__(iface.mainWindow())
        self.iface = iface
        self.plugin = plugin
        self.canvas = iface.mapCanvas()
        self.processor = None
        
        print("Initializing DronePrecisionMapperDialog...")
        self.setup_ui()
        print("DronePrecisionMapperDialog setup complete")
        
    def setup_ui(self):
        """Setup the user interface."""
        print("Setting up UI...")
        self.setWindowTitle("Drone Precision Mapper")
        self.setMinimumSize(500, 400)
        
        layout = QVBoxLayout()
        
        # Mode selection
        mode_group = QGroupBox("Processing Mode")
        mode_layout = QVBoxLayout()
        
        self.single_mode_radio = QRadioButton("Single Image")
        self.batch_mode_radio = QRadioButton("Batch Processing (Multiple Images)")
        self.single_mode_radio.setChecked(True)
        
        mode_layout.addWidget(self.single_mode_radio)
        mode_layout.addWidget(self.batch_mode_radio)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        print("Added mode selection group")
        
        # File selection
        file_group = QGroupBox("File Selection")
        file_layout = QVBoxLayout()
        
        # Single image selection
        single_layout = QHBoxLayout()
        self.single_file_edit = QLineEdit()
        self.single_file_edit.setPlaceholderText("Select a single drone image...")
        self.single_file_btn = QPushButton("Browse...")
        self.single_file_btn.clicked.connect(self.select_single_file)
        single_layout.addWidget(self.single_file_edit)
        single_layout.addWidget(self.single_file_btn)
        file_layout.addLayout(single_layout)
        
        # Batch folder selection
        batch_layout = QHBoxLayout()
        self.batch_folder_edit = QLineEdit()
        self.batch_folder_edit.setPlaceholderText("Select folder containing drone images...")
        self.batch_folder_btn = QPushButton("Browse...")
        self.batch_folder_btn.clicked.connect(self.select_batch_folder)
        batch_layout.addWidget(self.batch_folder_edit)
        batch_layout.addWidget(self.batch_folder_btn)
        file_layout.addLayout(batch_layout)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        print("Added file selection group")
        
        # Coordinate display (fallback)
        coord_group = QGroupBox("Coordinate Display (Fallback)")
        coord_layout = QVBoxLayout()
        
        self.coord_display = QTextEdit()
        self.coord_display.setMaximumHeight(100)
        self.coord_display.setReadOnly(True)
        self.coord_display.setPlaceholderText("Click on the image to see coordinates here...")
        coord_layout.addWidget(self.coord_display)
        
        # Clear button for dialog display
        clear_btn = QPushButton("Clear Display")
        clear_btn.clicked.connect(self.clear_dialog_display)
        coord_layout.addWidget(clear_btn)
        
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        print("Added coordinate display group")
        
        # Status
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        status_layout.addWidget(self.progress_bar)
        
        # Add cancel button for batch processing
        self.cancel_btn = QPushButton("Cancel Batch Processing")
        self.cancel_btn.clicked.connect(self.cancel_batch_processing)
        self.cancel_btn.setVisible(False)
        status_layout.addWidget(self.cancel_btn)
        
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(150)
        self.status_text.setReadOnly(True)
        status_layout.addWidget(self.status_text)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        print("Added status group")
        
        # Buttons
        button_layout = QHBoxLayout()
        
        self.process_btn = QPushButton("Process Images")
        self.process_btn.clicked.connect(self.process_images)
        self.process_btn.setEnabled(False)
        
        self.close_btn = QPushButton("Close")
        self.close_btn.clicked.connect(self.close)
        
        button_layout.addWidget(self.process_btn)
        button_layout.addWidget(self.close_btn)
        layout.addLayout(button_layout)
        
        print("Added buttons")
        
        self.setLayout(layout)
        
        # Connect mode changes
        self.single_mode_radio.toggled.connect(self.on_mode_changed)
        self.batch_mode_radio.toggled.connect(self.on_mode_changed)
        
        # Initial mode setup
        self.on_mode_changed()
        
        print("UI setup complete")
        
    def on_mode_changed(self):
        """Handle mode selection changes."""
        if self.single_mode_radio.isChecked():
            self.single_file_edit.setEnabled(True)
            self.single_file_btn.setEnabled(True)
            self.batch_folder_edit.setEnabled(False)
            self.batch_folder_btn.setEnabled(False)
        else:
            self.single_file_edit.setEnabled(False)
            self.single_file_btn.setEnabled(False)
            self.batch_folder_edit.setEnabled(True)
            self.batch_folder_btn.setEnabled(True)
            
        self.update_process_button()
        
    def select_single_file(self):
        """Select a single image file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Drone Image",
            "",
            "Image Files (*.jpg *.jpeg *.tiff *.tif *.png)"
        )
        if file_path:
            self.single_file_edit.setText(file_path)
            self.update_process_button()
            
    def select_batch_folder(self):
        """Select a folder for batch processing."""
        folder_path = QFileDialog.getExistingDirectory(
            self,
            "Select Folder with Drone Images"
        )
        if folder_path:
            self.batch_folder_edit.setText(folder_path)
            self.update_process_button()
            
    def update_process_button(self):
        """Update the process button state."""
        if self.single_mode_radio.isChecked():
            enabled = bool(self.single_file_edit.text().strip())
        else:
            enabled = bool(self.batch_folder_edit.text().strip())
            
        self.process_btn.setEnabled(enabled)
        
    def log_message(self, message):
        """Add a message to the status text."""
        self.status_text.append(message)
        
    def process_images(self):
        """Process the selected images."""
        try:
            if self.single_mode_radio.isChecked():
                self.process_single_image()
            else:
                self.process_batch_images()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Processing failed: {str(e)}")
            self.log_message(f"ERROR: {str(e)}")
            
    def process_single_image(self):
        """Process a single image."""
        image_path = self.single_file_edit.text().strip()
        if not Path(image_path).exists():
            QMessageBox.warning(self, "Warning", "Selected image file does not exist.")
            return
        self.log_message("Processing single image...")
        self.cleanup_existing_tools()
        from .image_processor import georeference_image, PrecisionClickTool, BasicCoordinatePickerTool
        result_params = georeference_image(image_path, self.log_message)
        if result_params == 'basic_picker':
            layer = QgsProject.instance().mapLayersByName(Path(image_path).stem)[-1]
            picker_tool = BasicCoordinatePickerTool(self.canvas, layer, self.update_coordinate_display)
            self.canvas.setExtent(layer.extent())
            self.canvas.refresh()
            self.canvas.setMapTool(picker_tool)
            self.plugin.active_tool = picker_tool
            self.log_message("ℹ️ This image is already georeferenced but lacks drone EXIF. Advanced correction is disabled, but you can still pick coordinates.")
            return
        if result_params:
            # --- Tag the layer with a persistent identifier ---
            layer = result_params['layer']
            layer.setCustomProperty("DronePrecisionMapper:source_image", str(image_path))
            try:
                with open(image_path, "rb") as f:
                    file_hash = hashlib.md5(f.read()).hexdigest()
                layer.setCustomProperty("DronePrecisionMapper:file_hash", file_hash)
            except Exception as e:
                self.log_message(f"⚠️ Could not compute file hash: {e}")
            # --------------------------------------------------
            precision_tool = PrecisionClickTool(self.canvas, result_params, self.update_coordinate_display, self.plugin.persistent_click_counters)
            self.canvas.setExtent(result_params['layer'].extent())
            self.canvas.refresh()
            self.canvas.setMapTool(precision_tool)
            self.canvas.refresh()
            self.plugin.active_tool = precision_tool
            self.log_message("✅ Single image processed successfully!")
            self.log_message("Precision click tool activated. Click on the image to get coordinates.")
            self.log_message("💡 Tip: Coordinates will appear in the 'Coordinate Display' dock widget.")
        else:
            self.log_message("❌ Single image processing failed.")

    def process_batch_images(self):
        """Process multiple images in batch."""
        folder_path = self.batch_folder_edit.text().strip()
        if not Path(folder_path).exists():
            QMessageBox.warning(self, "Warning", "Selected folder does not exist.")
            return
        self.log_message("Processing batch images...")
        self.cleanup_existing_tools()
        self.progress_bar.setVisible(True)
        self.cancel_btn.setVisible(True)
        self.progress_bar.setValue(0)
        from .image_processor import process_multiple_drone_photos, MultiLayerPrecisionClickTool
        processed_layers = process_multiple_drone_photos(folder_path, self.log_message, self.progress_bar)
        # --- Tag each processed layer with a persistent identifier ---
        for layer_data in processed_layers:
            image_path = layer_data['params']['image_path']
            layer = layer_data['params']['layer']
            layer.setCustomProperty("DronePrecisionMapper:source_image", str(image_path))
            try:
                with open(image_path, "rb") as f:
                    file_hash = hashlib.md5(f.read()).hexdigest()
                layer.setCustomProperty("DronePrecisionMapper:file_hash", file_hash)
            except Exception as e:
                self.log_message(f"⚠️ Could not compute file hash: {e}")
        # ------------------------------------------------------------
        self.progress_bar.setVisible(False)
        self.cancel_btn.setVisible(False)
        if processed_layers:
            multi_tool = MultiLayerPrecisionClickTool(self.canvas, processed_layers, self.update_coordinate_display, self.plugin.persistent_click_counters)
            all_extents = [layer_data['params']['layer'].extent() for layer_data in processed_layers]
            combined_extent = all_extents[0]
            for extent in all_extents[1:]:
                combined_extent.combineExtentWith(extent)
            self.canvas.setExtent(combined_extent)
            self.canvas.refresh()
            self.canvas.setMapTool(multi_tool)
            self.canvas.refresh()
            self.plugin.active_tool = multi_tool
            self.log_message(f"✅ Successfully processed {len(processed_layers)} images!")
            self.log_message("Multi-layer precision tool activated.")
            self.log_message("Use keyboard shortcuts: N (next), P (previous), L (list)")
            self.log_message("💡 Tip: Coordinates will appear in the 'Coordinate Display' dock widget.")
        else:
            self.log_message("❌ Batch processing failed.")

    def update_coordinate_display(self, message):
        """Update the coordinate display with new coordinates."""
        # Try to update the dock widget first
        if self.plugin.coordinate_dock:
            self.plugin.coordinate_dock.update_coordinates(message)
        
        # Also update the dialog display as fallback
        if hasattr(self, 'coord_display'):
            self.coord_display.append(message)
            # Auto-scroll to bottom
            cursor = self.coord_display.textCursor()
            cursor.movePosition(cursor.End)
            self.coord_display.setTextCursor(cursor)

    def cancel_batch_processing(self):
        """Cancel batch processing."""
        self.progress_bar.setValue(0)  # Reset progress bar to trigger cancellation
        self.progress_bar.setVisible(False)
        self.cancel_btn.setVisible(False)
        self.log_message("⏹️ Batch processing cancelled by user")

    def cleanup_existing_tools(self):
        """Clean up any existing tools and markers before activating a new one."""
        if self.plugin.active_tool:
            # Deactivate the current tool (this will clean up markers)
            self.canvas.unsetMapTool(self.plugin.active_tool)
            self.plugin.active_tool = None
        
        # Force canvas refresh to clear any remaining markers
        self.canvas.refresh()
        
        # Clear any existing layers that might be causing conflicts
        # This ensures we start with a clean slate
        self.log_message("🧹 Cleaned up previous tool and markers")
        
    def reset_click_counters(self):
        """Reset click counters in active tools when switching between images."""
        if hasattr(self.plugin.active_tool, 'click_counter'):
            self.plugin.active_tool.click_counter = 0 

    def clear_dialog_display(self):
        """Clear the dialog coordinate display."""
        self.coord_display.clear() 