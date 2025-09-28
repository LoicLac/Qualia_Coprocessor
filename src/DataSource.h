/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Fichier: DataSource.h
 * Rôle: Interface publique pour le module de Source de Données.
 */

#pragma once

#include <Arduino.h>

// La signature est modifiée pour "retourner" les handles des tâches créées
void start_data_source_tasks(TaskHandle_t* outLissajousHandle, TaskHandle_t* outSpiHandle);

// Déclaration de la variable globale pour l'initialisation SPI
extern bool g_spi_initialized_ok;