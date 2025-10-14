/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Fichier: DataSource.h
 * Rôle: Interface publique pour le module de Source de Données.
 */

#pragma once

#include <Arduino.h>

// Crée et démarre la tâche SPI listener
void start_data_source_tasks(TaskHandle_t* outSpiHandle);

// Déclaration de la variable globale pour l'initialisation SPI
extern bool g_spi_initialized_ok;