# Task 4.8: Property Test for Special Character Sanitization

## Status: ✓ COMPLETED

## Property Tested
**Property 23: Special character sanitization**
- **Validates: Requirements 7.2**

## Implementation Summary

### Property Statement
*For any* audio event name containing special characters (spaces, colons, underscores), the system should sanitize them to produce valid C++ identifiers.

### Test Implementation

The property test validates that the `CodeGenerator.sanitize_identifier()` method correctly handles all special characters and produces valid C++ identifiers.

#### Key Properties Tested:

1. **Valid C++ Identifier Output**
   - Output contains only alphanumeric characters and underscores
   - Output does not start with a digit
   - Output is never empty

2. **Deterministic Behavior**
   - Same input always produces same output
   - No randomness in sanitization

3. **No Consecutive Underscores**
   - Multiple underscores are collapsed to single underscore
   - Cleaner, more readable identifiers

4. **No Leading/Trailing Underscores**
   - Leading underscores only allowed when prefixing numeric strings
   - No trailing underscores (except for UNNAMED special case)

### Test Results

All property-based tests **PASSED** with 100 examples each:

```
[TEST 1] ✓ PASSED: Sanitized output is valid C++ identifier (100 examples)
[TEST 2] ✓ PASSED: Sanitization is deterministic (100 examples)
[TEST 3] ✓ PASSED: No consecutive underscores (100 examples)
[TEST 4] ✓ PASSED: No leading or trailing underscores (100 examples)
```

### Example Transformations

The sanitization correctly handles:

| Input | Output |
|-------|--------|
| `Play__Robot_Vic_Sfx__Blink` | `PLAY_ROBOT_VIC_SFX_BLINK` |
| `Event With Spaces` | `EVENT_WITH_SPACES` |
| `Event:With:Colons` | `EVENT_WITH_COLONS` |
| `Event-With-Dashes` | `EVENT_NEGWITH_NEGDASHES` |
| `Event.With.Dots` | `EVENT_DOTWITH_DOTDOTS` |
| `Event/With/Slashes` | `EVENT_SLASHWITH_SLASHSLASHES` |
| `Event(With)Parens` | `EVENT_LPARENWITH_RPARENPARENS` |
| `Event[With]Brackets` | `EVENT_LBRACKWITH_RBRACKBRACKETS` |
| `Event{With}Braces` | `EVENT_LBRACEWITH_RBRACEBRACES` |
| `Event<With>Angles` | `EVENT_LTWITH_GTANGLES` |
| `Event&With&Ampersands` | `EVENT_AMPWITH_AMPAMPERSANDS` |
| `Event\|With\|Pipes` | `EVENT_PIPEWITH_PIPEPIPES` |
| `Event!With!Bangs` | `EVENT_BANGWITH_BANGBANGS` |
| `Event?With?Questions` | `EVENT_QUESTWITH_QUESTQUESTIONS` |
| `Event@With@Ats` | `EVENT_ATWITH_ATATS` |
| `Event#With#Hashes` | `EVENT_HASHWITH_HASHHASHES` |
| `Event$With$Dollars` | `EVENT_DOLLARWITH_DOLLARDOLLARS` |
| `Event%With%Percents` | `EVENT_PERCENTWITH_PERCENTPERCENTS` |
| `Event+With+Plus` | `EVENT_PLUSWITH_PLUSPLUS` |
| `Event=With=Equals` | `EVENT_EQWITH_EQEQUALS` |
| `Event*With*Stars` | `EVENT_STARWITH_STARSTARS` |
| `123StartWithDigit` | `_123STARTWITHDIGIT` |
| `` (empty) | `UNNAMED` |
| `___` | `UNNAMED` |
| `Multiple___Underscores` | `MULTIPLE_UNDERSCORES` |

### Implementation Details

The `sanitize_identifier()` method in `code_generator.py`:

1. **Replaces special characters** with descriptive suffixes:
   - `-` → `_NEG`
   - `.` → `_DOT`
   - `/` → `_SLASH`
   - `(` → `_LPAREN`, `)` → `_RPAREN`
   - `[` → `_LBRACK`, `]` → `_RBRACK`
   - `{` → `_LBRACE`, `}` → `_RBRACE`
   - `<` → `_LT`, `>` → `_GT`
   - `&` → `_AMP`
   - `|` → `_PIPE`
   - `!` → `_BANG`
   - `?` → `_QUEST`
   - `@` → `_AT`
   - `#` → `_HASH`
   - `$` → `_DOLLAR`
   - `%` → `_PERCENT`
   - `+` → `_PLUS`
   - `=` → `_EQ`
   - `*` → `_STAR`
   - Spaces and colons → `_`

2. **Removes invalid characters** using regex

3. **Collapses consecutive underscores** to single underscore

4. **Removes leading/trailing underscores**

5. **Prefixes numeric-only strings** with underscore

6. **Handles empty strings** by returning "UNNAMED"

7. **Converts to uppercase** for consistency

### Test Files

- **Property test**: `test_property_special_character_sanitization.py`
- **Simple test**: `test_sanitization_simple.py`
- **PBT runner**: `run_pbt_sanitization.py`

### Validation

The property test confirms that:
- ✓ All special characters are properly sanitized
- ✓ Output is always a valid C++ identifier
- ✓ Sanitization is deterministic and consistent
- ✓ Edge cases (empty strings, numeric strings, special characters) are handled correctly
- ✓ Requirements 7.2 is fully satisfied

## Conclusion

Task 4.8 is **COMPLETE**. Property 23 (Special character sanitization) has been successfully validated through comprehensive property-based testing with 100 examples per property, ensuring robust handling of all special characters in audio event names.
