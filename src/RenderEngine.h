/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Fichier: RenderEngine.h
 * Rôle: Interface publique pour le module de Rendu et de Contrôle.
 */

#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "../shared_protocol.h"

extern QueueHandle_t points_queue;
extern QueueHandle_t raw_points_queue;

void setup_renderer_and_controls();
// Démarre les tâches de rendu et de contrôle
void start_render_and_control_tasks(TaskHandle_t spiHandle);