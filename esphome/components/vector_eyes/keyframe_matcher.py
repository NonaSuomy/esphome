"""
Keyframe-audio matching algorithm for Vector animations.

This module matches audio events to visual keyframes based on timing proximity.
"""
import logging
from typing import List, Dict, Optional, Tuple

try:
    from .keyframe_extractor import ProceduralFaceData
    from .animation_data import AudioEventData
except ImportError:
    from keyframe_extractor import ProceduralFaceData
    from animation_data import AudioEventData


logger = logging.getLogger(__name__)


class KeyframeMatcher:
    """
    Matches audio events to visual keyframes based on timing proximity.
    
    Uses a 100ms tolerance window for matching. Audio events that cannot be
    matched within this tolerance are logged as unmatched.
    """
    
    def __init__(self, tolerance_ms: int = 100):
        """
        Initialize the KeyframeMatcher.
        
        Args:
            tolerance_ms: Maximum time difference (in milliseconds) for matching
                         audio events to keyframes. Default is 100ms.
        """
        self.tolerance_ms = tolerance_ms
    
    def validate_assignments(
        self,
        keyframe_sound_map: Dict[int, str],
        audio_events: List[AudioEventData]
    ) -> Tuple[bool, List[str]]:
        """
        Validate audio assignments to ensure correctness.
        
        Checks:
        1. No keyframe has multiple audio events
        2. No audio event is assigned to multiple keyframes
        
        Args:
            keyframe_sound_map: Dictionary mapping keyframe index to sound filename
            audio_events: List of all audio events
        
        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []
        
        # Check 1: No keyframe should have multiple audio events
        # (This is enforced by dict structure, but we verify)
        keyframe_indices = list(keyframe_sound_map.keys())
        if len(keyframe_indices) != len(set(keyframe_indices)):
            errors.append("Duplicate keyframe indices found in mapping")
        
        # Check 2: No audio event should be assigned to multiple keyframes
        assigned_sounds = list(keyframe_sound_map.values())
        if len(assigned_sounds) != len(set(assigned_sounds)):
            # Find duplicates
            seen = set()
            duplicates = set()
            for sound in assigned_sounds:
                if sound in seen:
                    duplicates.add(sound)
                seen.add(sound)
            
            errors.append(f"Audio events assigned to multiple keyframes: {duplicates}")
        
        # Check 3: All assigned sounds should come from the audio events list
        audio_event_files = {ae.wav_file for ae in audio_events if ae.wav_file}
        assigned_sound_set = set(assigned_sounds)
        
        invalid_sounds = assigned_sound_set - audio_event_files
        if invalid_sounds:
            errors.append(f"Assigned sounds not in audio events: {invalid_sounds}")
        
        is_valid = len(errors) == 0
        return is_valid, errors
    
    def match_audio_to_keyframes(
        self,
        visual_keyframes: List[ProceduralFaceData],
        audio_events: List[AudioEventData]
    ) -> Tuple[Dict[int, str], List[AudioEventData]]:
        """
        Assign each audio event to its closest visual keyframe.
        
        Args:
            visual_keyframes: List of visual keyframes sorted by trigger time
            audio_events: List of audio events to match
        
        Returns:
            A tuple of:
            - Dictionary mapping keyframe index to sound filename
            - List of unmatched audio events (those beyond tolerance)
        """
        keyframe_sound_map = {}
        unmatched_events = []
        
        for audio_event in audio_events:
            closest_idx = None
            min_distance = float('inf')
            
            # Find the closest visual keyframe
            for idx, visual_kf in enumerate(visual_keyframes):
                distance = abs(visual_kf.trigger_time - audio_event.trigger_time)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = idx
            
            # Only assign if within tolerance (100ms by default)
            if min_distance <= self.tolerance_ms and closest_idx is not None:
                # Only assign if this keyframe doesn't already have audio
                if closest_idx not in keyframe_sound_map:
                    keyframe_sound_map[closest_idx] = audio_event.wav_file
                else:
                    # Keyframe already has audio, this is an unmatched event
                    logger.warning(
                        f"[WARN] KeyframeMatcher: Audio event at {audio_event.trigger_time}ms "
                        f"cannot be matched - keyframe already has audio"
                    )
                    unmatched_events.append(audio_event)
            else:
                # Audio event is beyond tolerance, log it as unmatched
                event_names_str = ", ".join(audio_event.event_names)
                logger.warning(
                    f"[WARN] KeyframeMatcher: Unmatched audio event '{event_names_str}' "
                    f"at trigger time {audio_event.trigger_time}ms "
                    f"(closest keyframe is {min_distance}ms away, tolerance is {self.tolerance_ms}ms)"
                )
                unmatched_events.append(audio_event)
        
        return keyframe_sound_map, unmatched_events
