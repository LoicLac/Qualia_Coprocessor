/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Project: Qualia-S3 Vector Scope
 * Version: 2.0 - SPI Refactoring Step 1
 *
 * Rôle: Chef d'orchestre.
 */

#include "RenderEngine.h"
#include "DataSource.h"
#include "../shared_protocol.h"

void setup() {
    DEBUG_INIT();

    // Les "handles" sont comme des télécommandes pour nos tâches.
    TaskHandle_t lissajousTaskHandle = NULL;
    TaskHandle_t spiTaskHandle = NULL;

    // 1. Initialiser le sous-système de rendu et de contrôle
    setup_renderer_and_controls();

    // 2. Créer les tâches de source de données et récupérer leurs handles
    start_data_source_tasks(&lissajousTaskHandle, &spiTaskHandle);

    // 3. Lancer les tâches de rendu et de contrôle, en leur donnant les handles
    // pour qu'elles puissent contrôler les sources de données.
    start_render_and_control_tasks(lissajousTaskHandle, spiTaskHandle);
}

void loop() {
  vTaskDelete(NULL);
}