/*
 * Handle buttons and helper functions
 *
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp32-hal-gpio.h>

#include "config.h"
#include "buttonsTask.h"


static TickType_t xButtonLastWakeTime;
static const TickType_t xButtonFrequency = pdMS_TO_TICKS(200);

static uint8_t oldButtons = 0;

EventGroupHandle_t buttonStatus;
#define BUTTON_1_PRESSED        0x0001
#define BUTTON_1_PRESSED_LONG   0x0002    // TODO: Not implemented
#define BUTTON_1_RELEASED       0x0004
#define BUTTON_2_PRESSED        0x0010
#define BUTTON_2_PRESSED_LONG   0x0020    // TODO: Not implemented
#define BUTTON_2_RELEASED       0x0040


void buttonInit(void)
{
  buttonStatus = xEventGroupCreate();
  configASSERT(buttonStatus); // DEBUG
  xEventGroupSetBits(buttonStatus, BUTTON_1_RELEASED);
  xEventGroupSetBits(buttonStatus, BUTTON_2_RELEASED);

  // Initialize buttons
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
}

// Task to read buttons and handle de-bounce etc.
// This task only updates the event group so use the helper functions
// to detect status (or wait for status change)
void buttonTask(void *params)
{
  buttonInit();

  xButtonLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    xTaskDelayUntil(&xButtonLastWakeTime, xButtonFrequency);

    if(digitalRead(BUTTON_1_PIN)) {
      // Not pressed
      xEventGroupClearBits(buttonStatus, BUTTON_1_PRESSED);
      xEventGroupSetBits(buttonStatus, BUTTON_1_RELEASED);
    } else {
      // Pressed
      xEventGroupClearBits(buttonStatus, BUTTON_1_RELEASED);
      xEventGroupSetBits(buttonStatus, BUTTON_1_PRESSED);
    }

    if(digitalRead(BUTTON_2_PIN)) {
      // Not pressed
      xEventGroupClearBits(buttonStatus, BUTTON_2_PRESSED);
      xEventGroupSetBits(buttonStatus, BUTTON_2_RELEASED);
    } else {
      // Pressed
      xEventGroupClearBits(buttonStatus, BUTTON_2_RELEASED);
      xEventGroupSetBits(buttonStatus, BUTTON_2_PRESSED);
    }
  }
}

// Alternative way to handle the buttons from interrupt
void IRAM_ATTR buttonInterrupt(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(digitalRead(BUTTON_1_PIN)) {
    xEventGroupClearBitsFromISR(buttonStatus, BUTTON_1_PRESSED);
    xEventGroupSetBitsFromISR(buttonStatus, BUTTON_1_RELEASED, &xHigherPriorityTaskWoken);
  } else {
    xEventGroupClearBitsFromISR(buttonStatus, BUTTON_1_RELEASED);
    xEventGroupSetBitsFromISR(buttonStatus, BUTTON_1_PRESSED, &xHigherPriorityTaskWoken);
  }
  if(digitalRead(BUTTON_2_PIN)) {
    xEventGroupClearBitsFromISR(buttonStatus, BUTTON_2_PRESSED);
    xEventGroupSetBitsFromISR(buttonStatus, BUTTON_2_RELEASED, &xHigherPriorityTaskWoken);
  } else {
    xEventGroupClearBitsFromISR(buttonStatus, BUTTON_2_RELEASED);
    xEventGroupSetBitsFromISR(buttonStatus, BUTTON_2_PRESSED, &xHigherPriorityTaskWoken);
  }
  if(&xHigherPriorityTaskWoken) portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Check if button 1 or button 2 are currently pressed
bool isButtonPressed(uint8_t which)
{
  if(which == 1)
    return (xEventGroupGetBits(buttonStatus) & BUTTON_1_PRESSED) ? true : false;
  else
    return (xEventGroupGetBits(buttonStatus) & BUTTON_2_PRESSED) ? true : false;
}

// Check if button 1 or button 2 are currently released
bool isButtonReleased(uint8_t which)
{
  if(which == 1)
    return (xEventGroupGetBits(buttonStatus) & BUTTON_1_RELEASED) ? true : false;
  else
    return (xEventGroupGetBits(buttonStatus) & BUTTON_2_RELEASED) ? true : false;
}

// Returns bitmask where bit 0x01 indicates button 1 pressed and 0x02 indicates button 2 pressed
uint8_t buttonPressStatus()
{
  uint8_t st;
  uint8_t bits = xEventGroupGetBits(buttonStatus) & 0xFF;
  st = (bits & BUTTON_1_PRESSED) ? 0x01 : 0x00;
  st |= (bits & BUTTON_2_PRESSED) ? 0x02 : 0x00;
  return st;
}

// Returns button pressed (bitmask 0x01 button 1, 0x02 button 2) only if the same
// bit was zero in *old, otherwise zero, i.e. button must be released
// before this considers it pressed. *old is updated with current status
uint8_t buttonPressed()
{
  uint8_t st = 0;
  uint8_t bits = xEventGroupGetBits(buttonStatus) & 0xFF;
  if((bits & BUTTON_1_PRESSED) && !(oldButtons & 0x01)) st |= 0x01;
  if((bits & BUTTON_2_PRESSED) && !(oldButtons & 0x02)) st |= 0x02;
  
  oldButtons = ((bits & BUTTON_1_PRESSED) ? 0x01 : 0x00) | ((bits & BUTTON_2_PRESSED) ? 0x02 : 0x00);

  return st;
}

// Wait for any button press pr until timeout
// Returns true if button was pressed or released before timeout passed, false if timeout happened
// Press is only considered press if button is not pressed in the "old" status
bool waitButtonPress(uint8_t *newbt, TickType_t timeout)
{
  EventBits_t bits = xEventGroupGetBits(buttonStatus);
  EventBits_t waitBits = 0;

  // Create the wake-up criteria
  if((bits & BUTTON_1_PRESSED) && (oldButtons & 0x01)) waitBits |= BUTTON_1_RELEASED;
  else waitBits |= BUTTON_1_PRESSED;
  if((bits & BUTTON_2_PRESSED) && (oldButtons & 0x02)) waitBits |= BUTTON_2_RELEASED;
  else waitBits |= BUTTON_2_PRESSED;

  bits = xEventGroupWaitBits(buttonStatus, waitBits, pdFALSE, pdFALSE, timeout);
  uint8_t st = 0;
  if((bits & BUTTON_1_PRESSED) && !(oldButtons & 0x01)) st |= 0x01;
  if((bits & BUTTON_2_PRESSED) && !(oldButtons & 0x02)) st |= 0x02;

  oldButtons = ((bits & BUTTON_1_PRESSED) ? 0x01 : 0x00) | ((bits & BUTTON_2_PRESSED) ? 0x02 : 0x00);

  if(bits & (BUTTON_1_PRESSED | BUTTON_2_PRESSED)) {  // One or the other button was considered to be pressed
    *newbt = st;
    return true;
  }

  // Timeout handled here
  *newbt = 0;
  return false;
}

// Wait until both buttons are released
// true = both were released, false = timeout
bool waitButtonRelease(TickType_t timeout)
{
  return xEventGroupWaitBits(buttonStatus, BUTTON_1_RELEASED | BUTTON_2_RELEASED, pdFALSE, pdTRUE, timeout) ==  (BUTTON_1_RELEASED | BUTTON_2_RELEASED);
}

// Wait until button press event or release event
// or until timeout. Return true if event happened, false if timeout
// If waiting for press and buttons are already pressed, returns immediately
bool waitButtonEvent(uint8_t which, bool press, TickType_t timeout)
{
  if(which == 1) {
    if(press)
      return (xEventGroupWaitBits(buttonStatus, BUTTON_1_PRESSED, pdFALSE, pdFALSE, timeout) & BUTTON_1_PRESSED) ? true : false;
    else
      return (xEventGroupWaitBits(buttonStatus, BUTTON_1_RELEASED, pdFALSE, pdFALSE, timeout) & BUTTON_1_RELEASED) ? true : false;
  } else if(which == 2) {
    if(press)
      return (xEventGroupWaitBits(buttonStatus, BUTTON_2_PRESSED, pdFALSE, pdFALSE, timeout) & BUTTON_2_PRESSED) ? true : false;
    else
      return (xEventGroupWaitBits(buttonStatus, BUTTON_2_RELEASED, pdFALSE, pdFALSE, timeout) & BUTTON_2_RELEASED) ? true : false;
  } else {
    if(press)
      return xEventGroupWaitBits(buttonStatus, BUTTON_1_PRESSED | BUTTON_2_PRESSED, pdFALSE, pdFALSE, timeout);
    else
      return xEventGroupWaitBits(buttonStatus, BUTTON_1_RELEASED | BUTTON_2_RELEASED, pdFALSE, pdFALSE, timeout);
  }
}

