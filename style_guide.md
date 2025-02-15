# Coding Style Guide

## 1. Naming Conventions

### Variables
- Use `snake_case` for all variables: `uint8_t led_brightness`
- Prefix global variables with `g_`: `g_tick`, `g_bitpos`
- Prefix class member variables with `m_`: `m_current_state`
- Constants should be `UPPER_CASE`: `TIMER_PRESCALER`, `NUM_LEDS`

### Functions
- Use `snake_case` for functions: `void update_acceleration()`
- Class methods use `snake_case`: `void set_brightness()`

### Types
- Use `PascalCase` for class names: `class LedDriver`
- Use `PascalCase` for struct names: `struct LedMapping`
- Use `PascalCase` for enum names: `enum class BlendLevel`
- Use `UPPER_CASE` for enum values: `BlendLevel::BLEND_0`

### Macros
- Use `UPPER_CASE` for all macros: `#define BCM_CYCLE_TICKS_SUM 263`

### File Names
- Use `snake_case` for all file names: `led_driver.h`, `fxpt_atan2.cpp`

## 2. Brace Style

Use the "One True Brace Style" (1TBS):

```cpp
if (condition) {
  // code
} else {
  // code
}

void function() {
  // code
}

class ClassName {
public:
  // code
};
```

## 3. File Organization

- Order of includes:
  1. Standard library headers
  2. External library headers
  3. Project-specific headers