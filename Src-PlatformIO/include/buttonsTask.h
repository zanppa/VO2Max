/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __BUTTONS_H__
#define __BUTTONS_H__


void buttonTask(void *params);

void buttonInit(void);
void IRAM_ATTR buttonInterrupt(void);

bool isButtonPressed(uint8_t which);
bool isButtonReleased(uint8_t which);
uint8_t buttonPressStatus();
uint8_t buttonPressed();

bool waitButtonPress(uint8_t *newbt, TickType_t timeout);
bool waitButtonRelease(TickType_t timeout);
bool waitButtonEvent(uint8_t which, bool press, TickType_t timeout);

#endif