/*
 * Copyright (C) 2024 Lauri Peltonen
 * GPL V3
 */

#ifndef __MENU_H__
#define __MENU_H__

typedef struct _floatNumParams_t {
  const char *label;
  float min;
  float max;
  float step;
} floatNumParams_t;

void doMenu(void *params);
void GetWeightkg(void);
void showParameters();

#endif