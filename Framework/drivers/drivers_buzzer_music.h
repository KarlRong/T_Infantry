/**
  ******************************************************************************
  * File Name          : drivers_buzzer_music.h
  * Description        : 蜂鸣器驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 蜂鸣器音乐表格
  ******************************************************************************
  */
#ifndef DRIVERS_BUZZER_MUSIC_H
#define DRIVERS_BUZZER_MUSIC_H

#include "drivers_buzzer_notes.h"

typedef struct {
	uint16_t note;
	uint16_t time;
}MusicNote;

MusicNote SuperMario[] = {
	{H3, 100}, {0, 50}, 
	{H3, 250}, {0, 50}, 
	{H3, 100}, {0, 50}, 
	{0, 150},
	{H1, 100}, {0, 50},  
	{H3, 250}, {0, 50},
	{H5, 250}, {0, 50},
	{0, 300},
	{M5, 250}, {0, 50},
	{0, 300},
	{H1, 250}, {0, 50}
};

MusicNote Intel[] = {
	{H1, 350}, {0, 50}, 
	{0, 200}, 
	{M5, 150}, {0, 50}, 
	{H1, 150}, {0, 50}, 
	{M5, 150}, {0, 50},  
	{H2, 200}
};

#endif
