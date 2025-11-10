# Crescendo (2024) - Kapok Example Implementation

This directory contains **example** implementations showing how to use Kapok for the 2024 Crescendo game. These files demonstrate different scoring patterns and autonomous task types.

## Purpose

Crescendo examples show:
- **IN_RANGE scoring** (speaker shots - shoot from various distances)
- **POSITIONAL scoring** (amp - precise positioning required)
- **Note tracking** (pickup and score cycles)
- Different task types for different game pieces

Compare with Reefscape (2025) to see how Kapok adapts to different games.

## Files

### CrescendoFieldConstants.java
Implements `FieldConstants` interface with 2024 field locations:
- **Speaker positions** (blue/red alliance)
- **Amp positions** (blue/red alliance)
- **Note locations** (centerline notes, spike notes)
- **AprilTag mappings** for vision alignment

### CrescendoTaskRegistryBuilder.java
Builds the task registry with Crescendo-specific tasks:
- `PICKUP:NOTE_C1` - Pick up centerline note 1
- `SCORE_SPEAKER:SPEAKER_BLUE` - Score in blue speaker (IN_RANGE pattern)
- `SCORE_AMP:AMP_BLUE` - Score in blue amp (POSITIONAL pattern)

### CrescendoContext.java
Extends `Context` with 2024 game state:
- Notes in robot count
- Picked up notes tracking
- Speaker vs Amp score counts
- Total score tracking

### CrescendoAutoBuilder.java
Builds autonomous commands from JSON routines using the TaskParser + TaskRegistry pattern.

**Example routine:**
```json
{
  "name": "4 Note Auto",
  "startingPose": [1.5, 5.55, 0],
  "tasks": [
    "PICKUP:NOTE_SPIKE_B2",
    "SCORE_SPEAKER:SPEAKER_BLUE",
    "PICKUP:NOTE_SPIKE_B1",
    "SCORE_SPEAKER:SPEAKER_BLUE",
    "PICKUP:NOTE_SPIKE_B3",
    "SCORE_SPEAKER:SPEAKER_BLUE"
  ]
}
```

## Key Differences from Reefscape

### Reefscape (2025)
- **POSITIONAL scoring only** - Must be at exact position to score coral
- **Vision alignment critical** - Precise positioning at reef branches
- **Location-based** - E2, F3, etc. (branches and levels)
- **Smart scoring** - Auto-select best available location

### Crescendo (2024)
- **IN_RANGE + POSITIONAL** - Speaker (flexible) vs Amp (precise)
- **Vision for aiming** - Not just positioning, but targeting
- **Note tracking** - Pickup cycles, note counting
- **Simpler locations** - Centerline notes, spike notes, scoring zones

## Implementation Status

These are **EXAMPLE** files showing the pattern. To make this fully functional:

1. **Add CrescendoStateMachine**
   - Similar to `ReefscapeStateMachine`
   - States: IDLE, PATHFINDING, VISION_ALIGNING, EXECUTING_PICKUP, EXECUTING_SCORE, etc.

2. **Add AutoExecutionCommand**
   - Command wrapper to run the state machine
   - Sets starting pose, runs periodic updates

3. **Implement subsystem integration**
   - Intake for picking up notes
   - Shooter for speaker shots
   - Arm/intake for amp scoring

4. **Add IN_RANGE scoring logic**
   - Use `ScoringHelper` utilities
   - Check distance, angle, shooter readiness
   - Vision-based aiming

## Using Crescendo Examples

To use these examples in your 2024 robot:

1. Copy the `Crescendo/` directory to your project
2. Update field coordinates in `CrescendoFieldConstants`
3. Implement `CrescendoStateMachine` with your subsystems
4. Create `AutoExecutionCommand` to run the state machine
5. Add IN_RANGE scoring strategy using `ScoringHelper`
6. Create JSON routine files in `deploy/Kapok/Crescendo/`

## Learning from Examples

**For future games:**
1. Start with field constants (locations + AprilTags)
2. Define task types (PICKUP, SCORE variants, etc.)
3. Build task registry with factory functions
4. Create game-specific context (state tracking)
5. Implement state machine with execution phases
6. Use `ScoringHelper` for location selection/sorting

The pattern is the same - only the specific tasks, locations, and scoring logic change per game!
