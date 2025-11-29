# Voice Command Architecture

## System Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         HUMAN-ROBOT INTERFACE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────────────┐         ┌──────────────────────────────────────┐ │
│   │  Unitree Controller  │         │         Go2 Built-in Mic             │ │
│   │    (Wireless)        │         │                                      │ │
│   └──────────┬───────────┘         └──────────────┬───────────────────────┘ │
│              │                                    │                         │
│              │ DDS                                │ DDS                     │
│              ▼                                    ▼                         │
│   ┌──────────────────────┐         ┌──────────────────────────────────────┐ │
│   │ /wirelesscontroller  │         │         /audiohub/data               │ │
│   │ unitree_go/          │         │         unitree_go/AudioData         │ │
│   │ WirelessController   │         │         (raw 16-bit PCM)             │ │
│   └──────────┬───────────┘         └──────────────┬───────────────────────┘ │
│              │                                    │                         │
│              │                                    │                         │
└──────────────┼────────────────────────────────────┼─────────────────────────┘
               │                                    │
               ▼                                    ▼
┌──────────────────────────────────┐  ┌──────────────────────────────────────┐
│    mission_executive_node        │  │       voice_command_node             │
│    (C++ FSM)                     │  │       (Python/Vosk)                  │
├──────────────────────────────────┤  ├──────────────────────────────────────┤
│                                  │  │                                      │
│  ┌────────────────────────────┐  │  │  ┌────────────────────────────────┐  │
│  │ Button Parser              │  │  │  │ State Monitor                  │  │
│  │ L2 + A = 0x0101            │  │  │  │ Only active in LISTENING       │  │
│  │ Rising edge detection      │  │  │  └────────────────────────────────┘  │
│  └────────────────────────────┘  │  │              │                       │
│              │                   │  │              ▼                       │
│              ▼                   │  │  ┌────────────────────────────────┐  │
│  ┌────────────────────────────┐  │  │  │ Audio Buffer                   │  │
│  │ State Machine              │  │  │  │ Resample to 16kHz if needed    │  │
│  │                            │  │  │  └────────────────────────────────┘  │
│  │  IDLE ──L2+A──▶ LISTENING  │  │  │              │                       │
│  │    ▲              │        │  │  │              ▼                       │
│  │    │          timeout/     │  │  │  ┌────────────────────────────────┐  │
│  │    │          target_obj   │  │  │  │ Vosk KaldiRecognizer           │  │
│  │    │              │        │  │  │  │ Grammar: [bottle, chair, ...]  │  │
│  │    │              ▼        │  │  │  │ ~50MB offline model            │  │
│  │  ...◀─── SEARCHING ───▶... │  │  │  └────────────────────────────────┘  │
│  └────────────────────────────┘  │  │              │                       │
│              │                   │  │              ▼                       │
│              │                   │  │  ┌────────────────────────────────┐  │
│              │                   │  │  │ Keyword Matcher                │  │
│              │                   │  │  │ Validates against vocabulary   │  │
│              │                   │  │  └────────────────────────────────┘  │
│              │                   │  │                                      │
└──────────────┼───────────────────┘  └──────────────┬───────────────────────┘
               │                                     │
               │ publishes                           │ publishes
               ▼                                     ▼
    ┌─────────────────────┐              ┌─────────────────────────┐
    │   /mission/state    │◀─────────────│  /mission/target_object │
    │   std_msgs/String   │   subscribes │  std_msgs/String        │
    │   "IDLE"            │              │  "bottle"               │
    │   "LISTENING"       │              │                         │
    │   "SEARCHING"       │              │                         │
    │   ...               │              │                         │
    └─────────────────────┘              └─────────────────────────┘
```

## Topic Flow

```
                    TRIGGER FLOW
                    ════════════
/wirelesscontroller ──▶ mission_executive_node
     (L2+A press)              │
                               ▼
                        State: LISTENING
                               │
                               ▼
                        /mission/state ──▶ voice_command_node
                               │                   │
                               │            (enables audio)
                               │                   │
                    VOICE FLOW │                   │
                    ══════════ │                   ▼
                               │          /audiohub/data
                               │                   │
                               │              [Vosk ASR]
                               │                   │
                               │                   ▼
                               │      /mission/target_object
                               │                   │
                               ◀───────────────────┘
                               │
                               ▼
                        State: SEARCHING
```

## Message Types

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/wirelesscontroller` | `unitree_go/WirelessController` | Go2 Robot | mission_executive |
| `/audiohub/data` | `unitree_go/AudioData` | Go2 Robot | voice_command |
| `/mission/state` | `std_msgs/String` | mission_executive | voice_command |
| `/mission/target_object` | `std_msgs/String` | voice_command | mission_executive |

## Trigger Logic

```
Controller keys (uint16 bitmask):
┌────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐
│ 15 │ 14 │ 13 │ 12 │ 11 │ 10 │  9 │  8 │  7 │  6 │  5 │  4 │  3 │  2 │  1 │  0 │
├────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤
│    │    │ R3 │ L3 │STRT│ SEL│ R2 │ L2 │ R1 │ L1 │DOWN│ UP │  Y │  X │  B │  A │
└────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘
                                    ▲                                        ▲
                                    │                                        │
                              Safety Enable                              Confirm
                                 (0x0100)                                (0x0001)

TRIGGER_MASK = L2 | A = 0x0101
```

