import json
import os
import re
import logging

try:
    from .code_generator import CodeGenerator
    from .keyframe_extractor import KeyframeExtractor, ProceduralFaceData
    from .keyframe_matcher import KeyframeMatcher
    from .audio_mapper import AudioMapper
    from .animation_data import AnimationData, KeyframeData, AudioEventData
except ImportError:
    from code_generator import CodeGenerator
    from keyframe_extractor import KeyframeExtractor, ProceduralFaceData
    from keyframe_matcher import KeyframeMatcher
    from audio_mapper import AudioMapper
    from animation_data import AnimationData, KeyframeData, AudioEventData

# Configure logging with structured format
# Format: [LEVEL] Component: Message (context)
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger('Converter')

INPUT_DIR = "animations_json"
OUTPUT_HEADER = "generated_animations.h"

def process_file(filepath, code_gen, audio_mapper, kf_extractor, kf_matcher):
    """
    Process a single animation JSON file and generate C++ code.
    
    Args:
        filepath: Path to JSON file
        code_gen: CodeGenerator instance
        audio_mapper: AudioMapper instance
        kf_extractor: KeyframeExtractor instance
        kf_matcher: KeyframeMatcher instance
        
    Returns:
        Tuple of (c_identifier, cpp_content, metadata) or None if processing fails
    """
    try:
        with open(filepath, "r") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse JSON file {filepath}: {e}")
        return None
    except Exception as e:
        logger.error(f"Failed to read file {filepath}: {e}")
        return None
    
    filename = os.path.basename(filepath)
    name_str = os.path.splitext(filename)[0]
    
    # Use the first key in the JSON object as the animation name
    # The JSON structure is { "anim_name": [ ... keyframes ... ] }
    try:
        anim_name_in_json = list(data.keys())[0]
        keyframes = data[anim_name_in_json]
    except (IndexError, KeyError) as e:
        logger.error(f"Invalid JSON structure in {filepath}: {e}")
        return None
    
    # Parse keyframes into AnimationData structure
    animation = AnimationData(name=name_str)
    
    for kf in keyframes:
        kf_type = kf.get("Name", "")
        trigger_time = kf.get("triggerTime_ms", 0)
        
        # Clamp negative trigger times to 0
        if trigger_time < 0:
            logger.warning(
                f"[WARN] Converter: Clamping negative trigger time {trigger_time}ms to 0ms "
                f"for {kf_type} in animation '{name_str}'"
            )
            trigger_time = 0
        
        if kf_type in ["ProceduralFaceKeyFrame", "RobotAudioKeyFrame", "HeadAngleKeyFrame", 
                       "LiftHeightKeyFrame", "RecordHeadingKeyFrame"]:
            keyframe_data = KeyframeData(
                trigger_time=trigger_time,
                keyframe_type=kf_type,
                data=kf
            )
            animation.keyframes.append(keyframe_data)
    
    # Extract procedural face keyframes
    face_keyframes = kf_extractor.extract_all_procedural_faces(animation)
    
    if not face_keyframes:
        logger.warning(f"No procedural face keyframes found in {name_str}")
        return None
    
    # Calculate animation duration
    if face_keyframes:
        animation.duration_ms = max(kf.trigger_time + kf.duration for kf in face_keyframes)
    
    # Clamp keyframe trigger times that exceed duration
    for kf in face_keyframes:
        if kf.trigger_time > animation.duration_ms:
            logger.warning(
                f"[WARN] Converter: Clamping keyframe trigger time {kf.trigger_time}ms to duration "
                f"{animation.duration_ms}ms in animation '{name_str}'"
            )
            kf.trigger_time = animation.duration_ms
    
    # Extract and map audio events
    audio_keyframes = animation.get_audio_keyframes()
    audio_events = []
    
    for akf in audio_keyframes:
        audio_names = kf_extractor.extract_audio_event(akf)
        if audio_names:
            # Map audio event to WAV file
            event_name = audio_names[0]
            wav_file = audio_mapper.map_event_to_wav(event_name)
            
            if wav_file:
                # Clamp audio event trigger time to valid range [0, duration]
                trigger_time = akf.trigger_time
                if trigger_time < 0:
                    logger.warning(
                        f"[WARN] Converter: Clamping negative audio trigger time {trigger_time}ms to 0ms "
                        f"for event '{event_name}' in animation '{name_str}'"
                    )
                    trigger_time = 0
                elif animation.duration_ms > 0 and trigger_time > animation.duration_ms:
                    logger.warning(
                        f"[WARN] Converter: Clamping audio trigger time {trigger_time}ms to duration "
                        f"{animation.duration_ms}ms for event '{event_name}' in animation '{name_str}'"
                    )
                    trigger_time = animation.duration_ms
                
                audio_event = AudioEventData(
                    trigger_time=trigger_time,
                    event_names=audio_names,
                    wav_file=wav_file
                )
                audio_events.append(audio_event)
                logger.info(f"Mapped audio event '{event_name}' -> '{wav_file}' at {trigger_time}ms")
    
    # Match audio events to keyframes
    keyframe_sound_map, unmatched = kf_matcher.match_audio_to_keyframes(face_keyframes, audio_events)
    
    # Log keyframe count
    logger.info(f"Processing animation '{name_str}': {len(face_keyframes)} keyframes, "
                f"{len(audio_events)} audio events, {len(unmatched)} unmatched")
    
    # Generate C++ code
    cpp_content, metadata = code_gen.generate_keyframe_array(
        name_str,
        face_keyframes,
        keyframe_sound_map,
        animation.duration_ms
    )
    
    return metadata.c_identifier, cpp_content, metadata



def main():
    """Main conversion function."""
    # TEMPORARY: Reduce animation count to fit in flash
    # Set to 1 to include all animations (requires SD card)
    # Set to 80 to include ~15 animations (fits in flash)
    ANIMATION_SKIP_RATIO = 80  # Keep every Nth animation
    
    # Initialize modules
    code_gen = CodeGenerator()
    audio_mapper = AudioMapper("audio_mappings.json")
    kf_extractor = KeyframeExtractor()
    kf_matcher = KeyframeMatcher(tolerance_ms=100)
    
    # These animations are ALWAYS included (used by code or critical for idle behavior)
    PRIORITY_ANIMATIONS = {
        # Core expressions (hardcoded in C++)
        "anim_eyes_look_happy",
        "anim_eyes_angry",
        "anim_eyes_awe",
        "anim_eyes_neutral",
        "anim_eyes_look_right",
        "anim_eyes_look_left",  # Will be generated from right
        
        # Keepalive/Blink (Vector's idle blinking)
        "anim_keepalive_blink_01",
        "anim_keepalive_eyesonly_loop_01",
        "anim_keepalive_eyesonly_loop_02",
        "anim_keepalive_eyesonly_loop_03",
        "anim_keepalive_eyesonly_loop_04",
        
        # Idle/Observing
        "anim_generic_look_up_01",
        "anim_generic_look_up_02",
        "anim_generic_look_up_idle_01",
        "anim_generic_look_up_idle_02",
        "anim_observing_around_subtle_01",
        "anim_observing_far_subtle_01",
        
        # Charger/Sleep
        "anim_rtsound_oncharger_asleep_front_01",
        "anim_rtsound_oncharger_observe_right_01",
        "anim_rtsound_oncharger_observe_front_01",
        "anim_observe_oncharger_getin_01",
        "anim_chargerdocking_comeoff_straight_03",
        
        # Dance Beat
        "anim_dancebeat_getin_01",
        "anim_dancebeat_getin_02",
        "anim_dancebeat_listening_01",
        "anim_dancebeat_listening_02",
        "anim_dancebeat_listening_03",
        "anim_dancebeat_quit_01",
        "anim_dancebeat_getready_01",
        
        # Explorer/Driving
        "anim_explorer_lookaround_01",
        "anim_hiking_lookaround_01",
        "anim_explorer_huh_far_01",
        "anim_explorer_huh_close_01",
        "anim_explorer_scan_left_01",
        "anim_explorer_scan_right_02",
        "anim_explorer_center_right_01",
        "anim_explorer_planning_getin_01",
        "anim_explorer_planning_getin_02",
        "anim_explorer_planning_getout_01",
        "anim_explorer_planning_getout_02",
        "anim_loco_driving01_start_01",
        "anim_loco_driving01_end_01",
        
        # Interactions
        "anim_handdetection_getin_01",
        "anim_handdetection_drive_loop_01",
        "anim_reacttocliff_stop_02",
        "anim_reacttohabitat_subtle_01",
        "anim_rtpickup_reaction_02",
        "anim_rtpickup_reaction_03",
        "anim_heldonpalm_getin_nervous_01",
        
        # Sleep/Wake
        "anim_gotosleep_getin_01",
        "anim_gotosleep_sleeping_01",
        
        # Extra Driving/Charger
        "anim_loco_driving01_start_02",
        "anim_rtsound_oncharger_observe_60right_01",
        
        # Eye contact (Vector's face tracking)
        "anim_eyecontact_lookloop_01",
        "anim_eyecontact_lookloop_02",
        
        # Pause/Idle
        "anim_pause_idle_01",
        "anim_pause_idle_02",
    }
    
    files = []
    if os.path.exists(INPUT_DIR):
        for f in os.listdir(INPUT_DIR):
            if f.endswith(".json"):
                files.append(os.path.join(INPUT_DIR, f))
    
    files.sort()

    # Generate header
    header_content = code_gen.generate_header()
    
    valid_animations = []  # List of (name_string, c_identifier, metadata)
    animation_counter = 0
    skipped_count = 0

    for filepath in files:
        animation_counter += 1
        name_str = os.path.splitext(os.path.basename(filepath))[0]
        
        # Always include priority animations
        is_priority = name_str in PRIORITY_ANIMATIONS
        
        # TEMPORARY: Skip animations to fit in flash (but keep priorities)
        if not is_priority and ANIMATION_SKIP_RATIO > 1 and animation_counter % ANIMATION_SKIP_RATIO != 0:
            skipped_count += 1
            continue
        
        result = process_file(filepath, code_gen, audio_mapper, kf_extractor, kf_matcher)
        if result:
            c_id, content, metadata = result
            header_content += content
            valid_animations.append((name_str, c_id, metadata))
        else:
            logger.warning(f"Skipped animation {name_str} due to processing errors")
            skipped_count += 1

    # Generate audio deduplication report
    header_content += code_gen.generate_audio_deduplication_report()
    
    # Generate AnimationData struct with metadata
    header_content += code_gen.generate_animation_data_struct()
    
    # Generate lookup table with PROGMEM
    header_content += code_gen.generate_lookup_table()
    
    # Generate helper functions
    header_content += code_gen.generate_helper_functions()
    
    # Generate footer
    header_content += code_gen.generate_footer()

    # Write output file
    with open(OUTPUT_HEADER, "w") as f:
        f.write(header_content)
    
    # Print summary
    total_files = len([f for f in os.listdir(INPUT_DIR) if f.endswith(".json")]) if os.path.exists(INPUT_DIR) else 0
    ratio_info = f" (reduced from {total_files} - keeping every {ANIMATION_SKIP_RATIO}th)" if ANIMATION_SKIP_RATIO > 1 else ""
    
    logger.info(f"Generated {OUTPUT_HEADER} with {len(valid_animations)} animations{ratio_info}.")
    logger.info(f"Skipped {skipped_count} animations")
    logger.info(f"Total unique audio files used: {len(code_gen.used_audio_files)}")
    
    # Print metadata summary
    total_frames = sum(m.frame_count for _, _, m in valid_animations)
    total_audio_events = sum(m.audio_event_count for _, _, m in valid_animations)
    logger.info(f"Total keyframes: {total_frames}")
    logger.info(f"Total audio events: {total_audio_events}")


if __name__ == "__main__":
    main()
