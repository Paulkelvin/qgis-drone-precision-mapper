#!/usr/bin/env python3
"""
Create a simple icon for the Drone Precision Mapper plugin
"""

from PIL import Image, ImageDraw, ImageFont
import os

def create_icon():
    """Create a simple drone icon for the plugin."""
    # Create a 32x32 image with a blue background
    size = 32
    img = Image.new('RGBA', (size, size), (0, 100, 200, 255))
    draw = ImageDraw.Draw(img)
    
    # Draw a simple drone shape (helicopter-like)
    # Main body (rectangle)
    draw.rectangle([8, 12, 24, 20], fill=(255, 255, 255, 255))
    
    # Rotor blades (lines)
    draw.line([(16, 8), (16, 12)], fill=(255, 255, 255, 255), width=2)
    draw.line([(12, 10), (20, 10)], fill=(255, 255, 255, 255), width=2)
    
    # Landing gear (small rectangles)
    draw.rectangle([10, 20, 12, 22], fill=(255, 255, 255, 255))
    draw.rectangle([20, 20, 22, 22], fill=(255, 255, 255, 255))
    
    # Save the icon
    icon_path = "DronePrecisionMapper/icon.png"
    img.save(icon_path)
    print(f"✅ Icon created: {icon_path}")
    
    return icon_path

if __name__ == "__main__":
    create_icon() 