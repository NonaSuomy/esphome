"""
Robust JSON parser for Vector animation files with comprehensive error handling.
"""
import json
import logging
from pathlib import Path
from typing import Optional

try:
    from .animation_data import AnimationData, KeyframeData, AudioEventData
except ImportError:
    from animation_data import AnimationData, KeyframeData, AudioEventData


logger = logging.getLogger(__name__)


class AnimationParseError(Exception):
    """Exception raised when animation parsing fails."""
    pass


class JSONParser:
    """Parser for Vector animation JSON files."""
    
    def parse_animation_file(self, filepath: str) -> Optional[AnimationData]:
        """
        Parse a JSON animation file and extract structured data.
        
        Args:
            filepath: Path to JSON file
            
        Returns:
            AnimationData containing name, keyframes, and metadata, or None if parsing fails
            
        Raises:
            AnimationParseError: If file cannot be parsed or is invalid
        """
        try:
            # Read and parse JSON
            with open(filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            logger.error(f"[ERROR] JSONParser: File not found: {filepath}")
            raise AnimationParseError(f"File not found: {filepath}")
        except json.JSONDecodeError as e:
            logger.error(f"[ERROR] JSONParser: Failed to parse {filepath} (line {e.lineno}: {e.msg})")
            raise AnimationParseError(f"Invalid JSON in {filepath}: {e.msg} at line {e.lineno}")
        except Exception as e:
            logger.error(f"[ERROR] JSONParser: Unexpected error reading {filepath}: {e}")
            raise AnimationParseError(f"Failed to read {filepath}: {e}")
        
        # Validate JSON structure
        if not isinstance(data, dict):
            logger.error(f"[ERROR] JSONParser: Root element must be an object in {filepath}")
            raise AnimationParseError(f"Invalid structure in {filepath}: root must be an object")
        
        if not data:
            logger.error(f"[ERROR] JSONParser: Empty JSON object in {filepath}")
            raise AnimationParseError(f"Empty animation data in {filepath}")
        
        # Extract animation name (first key in the JSON object)
        try:
            anim_name = list(data.keys())[0]
            keyframes_raw = data[anim_name]
        except (IndexError, KeyError) as e:
            logger.error(f"[ERROR] JSONParser: Cannot extract animation name from {filepath}")
            raise AnimationParseError(f"Invalid structure in {filepath}: cannot find animation data")
        
        # Validate keyframes array
        if not isinstance(keyframes_raw, list):
            logger.error(f"[ERROR] JSONParser: Keyframes must be an array in {filepath}")
            raise AnimationParseError(f"Invalid structure in {filepath}: keyframes must be an array")
        
        # Create AnimationData object
        animation = AnimationData(name=anim_name)
        
        # Parse keyframes
        for idx, kf_raw in enumerate(keyframes_raw):
            if not isinstance(kf_raw, dict):
                logger.warning(f"[WARN] JSONParser: Skipping non-object keyframe at index {idx} in {anim_name}")
                continue
            
            # Check for required "Name" field
            if "Name" not in kf_raw:
                logger.warning(f"[WARN] JSONParser: Skipping keyframe without 'Name' field at index {idx} in {anim_name}")
                continue
            
            keyframe_type = kf_raw["Name"]
            
            # Check for required "triggerTime_ms" field
            if "triggerTime_ms" not in kf_raw:
                logger.warning(f"[WARN] JSONParser: Skipping keyframe without 'triggerTime_ms' at index {idx} in {anim_name}")
                continue
            
            try:
                trigger_time = int(kf_raw["triggerTime_ms"])
            except (ValueError, TypeError):
                logger.warning(f"[WARN] JSONParser: Invalid triggerTime_ms at index {idx} in {anim_name}")
                continue
            
            # Create KeyframeData
            keyframe = KeyframeData(
                trigger_time=trigger_time,
                keyframe_type=keyframe_type,
                data=kf_raw
            )
            
            if not keyframe.validate():
                logger.warning(f"[WARN] JSONParser: Invalid keyframe at index {idx} in {anim_name}")
                continue
            
            animation.keyframes.append(keyframe)
            
            # If this is an audio keyframe, also create AudioEventData
            if keyframe_type == "RobotAudioKeyFrame":
                audio_names = kf_raw.get("audioName", [])
                
                # Handle case where audioName might be in eventGroups
                if not audio_names and "eventGroups" in kf_raw:
                    for group in kf_raw["eventGroups"]:
                        if isinstance(group, dict) and "audioName" in group:
                            audio_names.extend(group["audioName"])
                
                if audio_names:
                    audio_event = AudioEventData(
                        trigger_time=trigger_time,
                        event_names=audio_names if isinstance(audio_names, list) else [audio_names]
                    )
                    
                    if audio_event.validate():
                        animation.audio_events.append(audio_event)
        
        # Calculate duration from last keyframe
        if animation.keyframes:
            max_trigger_time = max(kf.trigger_time for kf in animation.keyframes)
            animation.duration_ms = max_trigger_time
        
        # Validate final animation data
        if not animation.validate():
            logger.error(f"[ERROR] JSONParser: Animation validation failed for {anim_name}")
            raise AnimationParseError(f"Invalid animation data for {anim_name}")
        
        logger.info(f"[INFO] JSONParser: Successfully parsed {anim_name} with {len(animation.keyframes)} keyframes")
        
        return animation
    
    def parse_animation_directory(self, directory: str) -> list:
        """
        Parse all JSON files in a directory.
        
        Args:
            directory: Path to directory containing JSON files
            
        Returns:
            List of AnimationData objects (skips files that fail to parse)
        """
        animations = []
        dir_path = Path(directory)
        
        if not dir_path.exists():
            logger.error(f"[ERROR] JSONParser: Directory not found: {directory}")
            return animations
        
        json_files = sorted(dir_path.glob("*.json"))
        
        for json_file in json_files:
            try:
                animation = self.parse_animation_file(str(json_file))
                if animation:
                    animations.append(animation)
            except AnimationParseError as e:
                logger.warning(f"[WARN] JSONParser: Skipping {json_file.name}: {e}")
                continue
            except Exception as e:
                logger.error(f"[ERROR] JSONParser: Unexpected error parsing {json_file.name}: {e}")
                continue
        
        logger.info(f"[INFO] JSONParser: Parsed {len(animations)} animations from {directory}")
        return animations
