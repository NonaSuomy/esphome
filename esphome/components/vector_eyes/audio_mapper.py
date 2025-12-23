"""
Audio event to WAV file mapping system.

This module provides the AudioMapper class which maps Vector's Wwise audio event names
to WAV file identifiers. It supports:
- Exact match lookups
- Pattern/substring matching for variants
- Specificity-based precedence (most specific match wins)
- External configuration file loading
- Hot-reload of mappings
"""

import json
import logging
from typing import Optional, Dict, List, Tuple
from pathlib import Path

logger = logging.getLogger(__name__)


class AudioMapper:
    """
    Maps audio event names to WAV file identifiers.
    
    Supports loading mappings from an external JSON configuration file with
    exact matches and pattern-based matching for audio event variants.
    """
    
    def __init__(self, mapping_file: Optional[str] = None):
        """
        Initialize AudioMapper with optional configuration file.
        
        Args:
            mapping_file: Path to JSON configuration file containing mappings.
                         If None, uses default empty mappings.
        """
        self.mapping_file = mapping_file
        self.exact_mappings: Dict[str, str] = {}
        self.pattern_mappings: List[Tuple[str, str]] = []  # (pattern, wav_file)
        
        if mapping_file:
            self.reload_mappings()
    
    def reload_mappings(self) -> bool:
        """
        Reload mappings from the configuration file.
        
        Preserves existing mappings if reload fails.
        
        Returns:
            True if reload was successful, False otherwise.
        """
        if not self.mapping_file:
            logger.warning("No mapping file configured, cannot reload")
            return False
        
        try:
            mapping_path = Path(self.mapping_file)
            if not mapping_path.exists():
                logger.error(f"Mapping file not found: {self.mapping_file}")
                return False
            
            with open(mapping_path, 'r') as f:
                config = json.load(f)
            
            # Load new mappings into temporary variables first
            new_exact_mappings = {}
            new_pattern_mappings = []
            
            # Load exact matches
            if 'exact_matches' in config:
                for event_name, wav_file in config['exact_matches'].items():
                    new_exact_mappings[event_name] = wav_file
                logger.info(f"Loaded {len(new_exact_mappings)} exact mappings")
            
            # Load pattern matches (sorted by specificity - longer patterns first)
            if 'pattern_matches' in config:
                patterns = []
                for pattern, wav_file in config['pattern_matches'].items():
                    patterns.append((pattern, wav_file))
                
                # Sort by pattern length (descending) for specificity precedence
                patterns.sort(key=lambda x: len(x[0]), reverse=True)
                new_pattern_mappings = patterns
                logger.info(f"Loaded {len(new_pattern_mappings)} pattern mappings")
            
            # Only update actual mappings after successful load
            self.exact_mappings = new_exact_mappings
            self.pattern_mappings = new_pattern_mappings
            
            logger.info(f"Successfully reloaded mappings from {self.mapping_file}")
            return True
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON syntax error in {self.mapping_file}: {e}")
            return False
        except Exception as e:
            logger.error(f"Error loading mappings from {self.mapping_file}: {e}")
            return False
    
    def map_event_to_wav(self, event_name: str) -> Optional[str]:
        """
        Map an audio event name to a WAV file identifier.
        
        Tries exact match first, then pattern matching with specificity precedence.
        
        Args:
            event_name: The Wwise audio event name (e.g., "Play__Robot_Vic_Sfx__Blink")
        
        Returns:
            WAV filename without extension (e.g., "blink"), or None if unmapped.
        """
        if not event_name:
            return None
        
        # Try exact match first
        if event_name in self.exact_mappings:
            wav_file = self.exact_mappings[event_name]
            logger.debug(f"Exact match: {event_name} -> {wav_file}")
            return wav_file
        
        # Try pattern matching (already sorted by specificity)
        for pattern, wav_file in self.pattern_mappings:
            if pattern in event_name:
                logger.debug(f"Pattern match: {event_name} contains '{pattern}' -> {wav_file}")
                return wav_file
        
        # No mapping found
        logger.warning(f"No mapping found for audio event: {event_name}")
        return None
    
    def find_best_match(self, event_name: str, patterns: List[str]) -> Optional[str]:
        """
        Find the most specific pattern that matches the event name.
        
        Args:
            event_name: The audio event name to match
            patterns: List of pattern strings to check
        
        Returns:
            The most specific (longest) matching pattern, or None if no match.
        """
        if not event_name or not patterns:
            return None
        
        # Find all matching patterns
        matches = [p for p in patterns if p in event_name]
        
        if not matches:
            return None
        
        # Return the longest (most specific) match
        return max(matches, key=len)
    
    def add_exact_mapping(self, event_name: str, wav_file: str):
        """
        Add or update an exact mapping.
        
        Args:
            event_name: The audio event name
            wav_file: The WAV file identifier (without extension)
        """
        self.exact_mappings[event_name] = wav_file
        logger.debug(f"Added exact mapping: {event_name} -> {wav_file}")
    
    def add_pattern_mapping(self, pattern: str, wav_file: str):
        """
        Add a pattern mapping.
        
        Args:
            pattern: The pattern to match (substring)
            wav_file: The WAV file identifier (without extension)
        """
        # Insert maintaining specificity order (longer patterns first)
        inserted = False
        for i, (existing_pattern, _) in enumerate(self.pattern_mappings):
            if len(pattern) > len(existing_pattern):
                self.pattern_mappings.insert(i, (pattern, wav_file))
                inserted = True
                break
        
        if not inserted:
            self.pattern_mappings.append((pattern, wav_file))
        
        logger.debug(f"Added pattern mapping: '{pattern}' -> {wav_file}")
    
    def get_mapping_stats(self) -> Dict[str, int]:
        """
        Get statistics about loaded mappings.
        
        Returns:
            Dictionary with 'exact_count' and 'pattern_count' keys.
        """
        return {
            'exact_count': len(self.exact_mappings),
            'pattern_count': len(self.pattern_mappings)
        }
