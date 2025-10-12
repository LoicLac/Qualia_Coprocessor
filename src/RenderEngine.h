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

void setup_renderer_and_controls();
// La signature est modifiée pour accepter les handles des tâches de données
void start_render_and_control_tasks(TaskHandle_t lissajousHandle, TaskHandle_t spiHandle);