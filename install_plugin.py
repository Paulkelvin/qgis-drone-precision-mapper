#!/usr/bin/env python3
"""
Installation script for Drone Precision Mapper QGIS Plugin
"""

import os
import sys
import shutil
from pathlib import Path

def get_qgis_plugins_path():
    """Get the QGIS plugins directory path."""
    if sys.platform == "win32":
        # Windows
        appdata = os.environ.get('APPDATA', '')
        return Path(appdata) / "QGIS" / "QGIS3" / "profiles" / "default" / "python" / "plugins"
    elif sys.platform == "darwin":
        # macOS
        home = os.path.expanduser("~")
        return Path(home) / "Library" / "Application Support" / "QGIS" / "QGIS3" / "profiles" / "default" / "python" / "plugins"
    else:
        # Linux
        home = os.path.expanduser("~")
        return Path(home) / ".local" / "share" / "QGIS" / "QGIS3" / "profiles" / "default" / "python" / "plugins"

def install_plugin():
    """Install the Drone Precision Mapper plugin."""
    # Get current directory (where this script is located)
    current_dir = Path(__file__).parent
    plugin_source = current_dir / "DronePrecisionMapper"
    
    if not plugin_source.exists():
        print("❌ Error: DronePrecisionMapper folder not found!")
        print(f"Expected location: {plugin_source}")
        return False
    
    # Get QGIS plugins directory
    plugins_dir = get_qgis_plugins_path()
    
    if not plugins_dir.exists():
        print(f"❌ Error: QGIS plugins directory not found!")
        print(f"Expected location: {plugins_dir}")
        print("\nPlease ensure QGIS is installed and has been run at least once.")
        return False
    
    # Target plugin directory
    plugin_target = plugins_dir / "DronePrecisionMapper"
    
    try:
        # Remove existing installation if it exists
        if plugin_target.exists():
            shutil.rmtree(plugin_target)
            print(f"🗑️  Removed existing installation from: {plugin_target}")
        
        # Copy plugin files
        shutil.copytree(plugin_source, plugin_target)
        print(f"✅ Plugin installed successfully to: {plugin_target}")
        
        print("\n🎉 Installation Complete!")
        print("\nNext steps:")
        print("1. Restart QGIS")
        print("2. Go to Plugins → Manage and Install Plugins")
        print("3. Find 'Drone Precision Mapper' and enable it")
        print("4. The plugin will appear in the Plugins menu")
        
        return True
        
    except Exception as e:
        print(f"❌ Installation failed: {str(e)}")
        return False

def main():
    """Main installation function."""
    print("🚁 Drone Precision Mapper QGIS Plugin Installer")
    print("=" * 50)
    
    # Check if running in QGIS environment
    try:
        from qgis.core import QgsApplication
        print("✅ Running in QGIS environment")
    except ImportError:
        print("⚠️  Not running in QGIS environment (this is normal)")
    
    # Install the plugin
    success = install_plugin()
    
    if success:
        print("\n✅ Installation completed successfully!")
    else:
        print("\n❌ Installation failed. Please check the error messages above.")
        sys.exit(1)

if __name__ == "__main__":
    main() 