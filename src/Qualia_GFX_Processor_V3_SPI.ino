/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Project: Qualia-S3 Vector Scope
 * Version: 3.0 - SPI Only (Lissajous removed)
 *
 * Rôle: Chef d'orchestre.
 */

#include "RenderEngine.h"
#include "DataSource.h"
#include "../shared_protocol.h"

void setup() {
    DEBUG_INIT();

    // Handle pour la tâche SPI
    TaskHandle_t spiTaskHandle = NULL;

    // 1. Initialiser le sous-système de rendu et de contrôle
    setup_renderer_and_controls();

    // 2. Créer la tâche de source de données SPI
    start_data_source_tasks(&spiTaskHandle);

    // 3. Lancer les tâches de rendu et de contrôle
    start_render_and_control_tasks(spiTaskHandle);
}

void loop() {
  vTaskDelete(NULL);
}