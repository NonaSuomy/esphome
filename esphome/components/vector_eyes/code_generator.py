"""
C++ code generation for Vector animations.

This module generates optimized C++ header files with:
- PROGMEM declarations for flash storage
- Compact AnimationKeyframe structs
- Efficient lookup tables
- Audio data deduplication
- Complete metadata
"""
import logging
import re
from typing import List, Dict, Set, Optional, Tuple
from dataclasses import dataclass

try:
    from .keyframe_extractor import ProceduralFaceData
    from .animation_data import AnimationData
except ImportError:
    from keyframe_extractor import ProceduralFaceData
    from animation_data import AnimationData


logger = logging.getLogger(__name__)


@dataclass
class AnimationMetadata:
    """Metadata for a generated animation."""
    name: str
    c_identifier: str
    frame_count: int
    duration_ms: int
    audio_event_count: int


class CodeGenerator:
    """
    Generates optimized C++ code for Vector animations.
    
    Features:
    - PROGMEM storage for all arrays
    - Compact AnimationKeyframe structs
    - Audio data deduplication
    - Complete metadata generation
    - Special character sanitization
    - Trigger time clamping
    """
    
    def __init__(self):
        """Initialize the code generator."""
        self.used_audio_files: Set[str] = set()
        self.animation_metadata: List[AnimationMetadata] = []
    
    def sanitize_identifier(self, name: str) -> str:
        """
        Sanitize a string to be a valid C++ identifier.
        
        Handles spaces, colons, underscores, and other special characters.
        
        Args:
            name: String to sanitize
            
        Returns:
            Valid C++ identifier
        """
        # Replace special characters with underscores
        # Handle common patterns in audio event names
        sanitized = name.replace(" ", "_")
        sanitized = sanitized.replace(":", "_")
        sanitized = sanitized.replace("-", "_NEG")
        sanitized = sanitized.replace(".", "_DOT")
        sanitized = sanitized.replace("/", "_SLASH")
        sanitized = sanitized.replace("\\", "_BACKSLASH")
        sanitized = sanitized.replace("(", "_LPAREN")
        sanitized = sanitized.replace(")", "_RPAREN")
        sanitized = sanitized.replace("[", "_LBRACK")
        sanitized = sanitized.replace("]", "_RBRACK")
        sanitized = sanitized.replace("{", "_LBRACE")
        sanitized = sanitized.replace("}", "_RBRACE")
        sanitized = sanitized.replace("<", "_LT")
        sanitized = sanitized.replace(">", "_GT")
        sanitized = sanitized.replace("&", "_AMP")
        sanitized = sanitized.replace("|", "_PIPE")
        sanitized = sanitized.replace("^", "_CARET")
        sanitized = sanitized.replace("~", "_TILDE")
        sanitized = sanitized.replace("`", "_BACKTICK")
        sanitized = sanitized.replace("'", "_QUOTE")
        sanitized = sanitized.replace('"', "_DQUOTE")
        sanitized = sanitized.replace(",", "_COMMA")
        sanitized = sanitized.replace(";", "_SEMI")
        sanitized = sanitized.replace("!", "_BANG")
        sanitized = sanitized.replace("?", "_QUEST")
        sanitized = sanitized.replace("@", "_AT")
        sanitized = sanitized.replace("#", "_HASH")
        sanitized = sanitized.replace("$", "_DOLLAR")
        sanitized = sanitized.replace("%", "_PERCENT")
        sanitized = sanitized.replace("+", "_PLUS")
        sanitized = sanitized.replace("=", "_EQ")
        sanitized = sanitized.replace("*", "_STAR")
        
        # Remove any remaining non-alphanumeric characters except underscores
        sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', sanitized)
        
        # Remove consecutive underscores
        sanitized = re.sub(r'_+', '_', sanitized)
        
        # Remove leading/trailing underscores
        sanitized = sanitized.strip('_')
        
        # Ensure it doesn't start with a digit (do this after removing leading underscores)
        if sanitized and sanitized[0].isdigit():
            sanitized = "_" + sanitized
        
        # Ensure it's not empty
        if not sanitized:
            sanitized = "UNNAMED"
        
        return sanitized.upper()
    
    def clamp_trigger_time(self, trigger_time: int, duration_ms: int) -> Tuple[int, bool]:
        """
        Clamp trigger time to valid range [0, duration].
        
        Args:
            trigger_time: Original trigger time in milliseconds
            duration_ms: Total animation duration in milliseconds
            
        Returns:
            Tuple of (clamped_time, was_clamped)
        """
        was_clamped = False
        clamped_time = trigger_time
        
        if trigger_time < 0:
            logger.warning(
                f"[WARN] CodeGenerator: Clamping negative trigger time {trigger_time}ms to 0ms"
            )
            clamped_time = 0
            was_clamped = True
        elif trigger_time > duration_ms:
            logger.warning(
                f"[WARN] CodeGenerator: Clamping trigger time {trigger_time}ms to duration {duration_ms}ms"
            )
            clamped_time = duration_ms
            was_clamped = True
        
        return clamped_time, was_clamped
    
    def generate_keyframe_array(
        self,
        animation_name: str,
        keyframes: List[ProceduralFaceData],
        keyframe_sound_map: Dict[int, str],
        duration_ms: int
    ) -> Tuple[str, AnimationMetadata]:
        """
        Generate C++ array for animation keyframes with PROGMEM storage.
        
        Generates compact AnimationKeyframe structs with:
        - PROGMEM storage for flash memory
        - Optimized struct layout (48 bytes per frame)
        - Inline face parameters (no pointer indirection)
        
        Args:
            animation_name: Name of the animation
            keyframes: List of procedural face keyframes
            keyframe_sound_map: Mapping of keyframe index to sound filename
            duration_ms: Total animation duration
            
        Returns:
            Tuple of (C++ code string, AnimationMetadata)
        """
        c_identifier = self.sanitize_identifier(animation_name)
        
        # Count audio events
        audio_event_count = len(keyframe_sound_map)
        
        # Track which audio files are used (for deduplication)
        for sound_file in keyframe_sound_map.values():
            if sound_file:
                self.used_audio_files.add(sound_file)
        
        # Generate array header with PROGMEM for flash storage
        cpp_content = f"// Animation: {animation_name}\n"
        cpp_content += f"// Frames: {len(keyframes)}, Duration: {duration_ms}ms, Audio Events: {audio_event_count}\n"
        cpp_content += f"// Memory: {len(keyframes) * 60} bytes (60 bytes per keyframe)\n"
        cpp_content += f"static const AnimationKeyframe {c_identifier}[] PROGMEM = {{\n"
        
        # Generate compact keyframe entries
        for i, kf in enumerate(keyframes):
            # Clamp trigger time to valid range [0, duration]
            trigger_time, was_clamped = self.clamp_trigger_time(kf.trigger_time, duration_ms)
            
            # Get assigned sound for this keyframe
            sound_str = "nullptr"
            if i in keyframe_sound_map:
                sound_file = keyframe_sound_map[i]
                if sound_file:
                    sound_str = f'"{sound_file}"'
            
            # Scale center coordinates for 128x64 display (Vector's was ~184x96)
            # Reduce by ~0.5x to fit smaller display
            center_x = kf.center_x * 0.5
            center_y = kf.center_y * 0.5
            
            # Generate compact keyframe struct with inline face parameters
            # Format: { trigger_time, duration, { face_params }, sound_name }
            cpp_content += f"  {{ {trigger_time}, {kf.duration}, "
            cpp_content += f"{{ {kf.scale_x}f, {kf.scale_y}f, {kf.angle}f, "
            cpp_content += f"{center_x}f, {center_y}f, "
            cpp_content += f"{kf.left_lid_top}f, {kf.left_lid_bottom}f, "
            cpp_content += f"{kf.right_lid_top}f, {kf.right_lid_bottom}f }}, "
            cpp_content += f"{sound_str} }},\n"
        
        cpp_content += "};\n\n"
        
        # Create metadata
        metadata = AnimationMetadata(
            name=animation_name,
            c_identifier=c_identifier,
            frame_count=len(keyframes),
            duration_ms=duration_ms,
            audio_event_count=audio_event_count
        )
        
        self.animation_metadata.append(metadata)
        
        return cpp_content, metadata
    
    def generate_animation_data_struct(self) -> str:
        """
        Generate AnimationData struct definition with complete metadata.
        
        The struct includes:
        - Animation name for lookup
        - Pointer to keyframe array (stored in PROGMEM)
        - Frame count for bounds checking
        - Total duration for validation
        - Audio event count for debugging
        
        Returns:
            C++ code string for struct definition
        """
        cpp_content = """
// Compact animation metadata structure
struct AnimationData {
  const char* name;                    // Animation identifier
  const AnimationKeyframe* frames;     // Pointer to PROGMEM keyframe array
  size_t length;                       // Number of keyframes
  uint32_t duration_ms;                // Total animation duration
  size_t audio_event_count;            // Number of audio events
};

"""
        return cpp_content
    
    def generate_lookup_table(self) -> str:
        """
        Generate efficient lookup table for all animations with PROGMEM.
        
        Creates a compact array of AnimationData structs stored in flash memory.
        The table is sorted alphabetically by name for potential binary search.
        
        Returns:
            C++ code string for lookup table
        """
        # Sort animations alphabetically for efficient lookup
        sorted_metadata = sorted(self.animation_metadata, key=lambda m: m.name)
        
        cpp_content = "// Efficient lookup table stored in PROGMEM (flash memory)\n"
        cpp_content += f"// Total animations: {len(sorted_metadata)}\n"
        cpp_content += "static const AnimationData ANIMATIONS[] PROGMEM = {\n"
        
        for metadata in sorted_metadata:
            cpp_content += f"  {{ \"{metadata.name}\", {metadata.c_identifier}, "
            cpp_content += f"{metadata.frame_count}, {metadata.duration_ms}, "
            cpp_content += f"{metadata.audio_event_count} }},\n"
        
        cpp_content += "};\n\n"
        
        # Add constant for animation count (more efficient than sizeof calculation)
        cpp_content += f"static constexpr size_t ANIMATION_COUNT = {len(sorted_metadata)};\n\n"
        
        return cpp_content
    
    def generate_helper_functions(self) -> str:
        """
        Generate efficient helper functions for animation lookup.
        
        Provides:
        - Linear search for animation data (O(n) but simple)
        - Metadata lookup for validation
        - Animation count accessor
        
        Note: Binary search could be added if lookup performance becomes critical,
        but linear search is sufficient for <100 animations.
        
        Returns:
            C++ code string for helper functions
        """
        cpp_content = """
// Efficient animation lookup functions

// Get animation keyframe data by name
// Returns pointer to PROGMEM keyframe array and sets length
inline const AnimationKeyframe* get_animation_data(const char* name, size_t* length) {
  for (size_t i = 0; i < ANIMATION_COUNT; i++) {
    if (strcmp(ANIMATIONS[i].name, name) == 0) {
      *length = ANIMATIONS[i].length;
      return ANIMATIONS[i].frames;
    }
  }
  *length = 0;
  return nullptr;
}

// Get animation metadata by name
// Returns pointer to AnimationData struct or nullptr if not found
inline const AnimationData* get_animation_metadata(const char* name) {
  for (size_t i = 0; i < ANIMATION_COUNT; i++) {
    if (strcmp(ANIMATIONS[i].name, name) == 0) {
      return &ANIMATIONS[i];
    }
  }
  return nullptr;
}

// Get total number of animations
inline size_t get_animation_count() {
  return ANIMATION_COUNT;
}

"""
        return cpp_content
    
    def generate_header(self) -> str:
        """
        Generate complete C++ header file with documentation.
        
        Returns:
            C++ header file content
        """
        header_content = """#pragma once
// Generated animation data for Vector robot eyes
// 
// This file contains:
// - Compact AnimationKeyframe structs (60 bytes each)
// - PROGMEM storage for all arrays (stored in flash, not RAM)
// - Efficient lookup tables with metadata
// - Audio data deduplication
//
// Memory layout:
// - AnimationKeyframe: 60 bytes on ESP32
//   - trigger_time: 4 bytes
//   - duration: 4 bytes
//   - ProceduralFace: 48 bytes (12 floats: 9 animation + 3 dimensions)
//   - sound_name: 4 bytes (pointer)
//
// All animation data is stored in PROGMEM (flash memory) to conserve RAM.
// Use get_animation_data() to retrieve animation keyframes.

#include "animation_player.h"
#include <cstring>
#include <algorithm>
#include <string>

namespace esphome {
namespace vector_eyes {

"""
        return header_content
    
    def generate_footer(self) -> str:
        """
        Generate C++ header footer.
        
        Returns:
            C++ footer content
        """
        footer_content = """} // namespace vector_eyes
} // namespace esphome
"""
        return footer_content
    
    def generate_audio_deduplication_report(self) -> str:
        """
        Generate a report of deduplicated audio files.
        
        Returns:
            Report string
        """
        report = f"\n// Audio Deduplication Report\n"
        report += f"// Total unique audio files used: {len(self.used_audio_files)}\n"
        report += f"// Audio files: {', '.join(sorted(self.used_audio_files))}\n\n"
        
        return report
    
    def reset(self):
        """Reset the generator state for a new generation run."""
        self.used_audio_files.clear()
        self.animation_metadata.clear()
