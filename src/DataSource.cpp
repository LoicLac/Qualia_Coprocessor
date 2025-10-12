/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Fichier: DataSource.cpp
 * Rôle: Implémentation finale des sources de données commutables (Lissajous & SPI).
 * Version: 2.0 - SPI Slave Implementation
 */

#include "DataSource.h"
#include "RenderEngine.h"
#include <math.h>

// Inclusions directes de l'ESP-IDF pour le pilote SPI esclave de bas niveau
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"

// --- VARIABLES PRIVÉES AU MODULE ---

static const int SCREEN_WIDTH = 720;
static const int SCREEN_HEIGHT = 720;
extern volatile float g_lissajous_speed; // Défini et contrôlé dans RenderEngine
volatile unsigned long g_last_spi_packet_time = 0;

// Variable globale pour indiquer si le SPI a été initialisé correctement
bool g_spi_initialized_ok = false;

// Brochage SPI
#define SPI_HOST_ID SPI2_HOST // Le contrôleur matériel SPI à utiliser
#define GPIO_MOSI   7
#define GPIO_MISO   6
#define GPIO_SCLK   5
#define GPIO_CS     15

// Buffer pour la réception DMA. Doit être aligné en mémoire.
// WORD_ALIGNED_ATTR est une macro de l'ESP-IDF qui garantit cet alignement.
static WORD_ALIGNED_ATTR uint8_t spi_rx_buffer[sizeof(PlotterPacket)];

// =======================================================
// ISR (Interrupt Service Routine) pour le SPI
// =======================================================
// Cette fonction est appelée par le pilote SPI à la fin de chaque transaction.
// Elle s'exécute dans un contexte d'interruption et doit être très rapide.
void IRAM_ATTR post_trans_cb(spi_slave_transaction_t *trans) {
    // Copier les données du buffer de réception DMA vers notre structure
    PlotterPacket packet;
    memcpy(&packet, spi_rx_buffer, sizeof(PlotterPacket));

    // Mettre à jour le timestamp pour le "heartbeat" visuel
    // Ceci est fait directement dans l'ISR pour une réactivité maximale à la réception.
    g_last_spi_packet_time = millis();

    // Envoyer le paquet à la file d'attente de la tâche de rendu
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (points_queue != NULL) { // Vérifier que la queue a été créée
        xQueueSendFromISR(points_queue, &packet, &xHigherPriorityTaskWoken);
    }
    // Noter: Pas de gestion d'erreur explicite ici comme demandé.
}

// =======================================================
// TÂCHE 1 : Producteur Lissajous
// =======================================================
static void lissajous_task(void *pvParameters) {
    float lissajous_time = 0.0;
    float lissajous_freq_a = 3.0, lissajous_freq_b = 4.0;
    float lissajous_phase = M_PI / 2.0;
    for (;;) {
        vTaskDelay(1 / portTICK_PERIOD_MS); // Délai pour simuler une cadence
        lissajous_time += g_lissajous_speed;
        float amplitude = SCREEN_WIDTH / 2.0;
        PlotterPacket packet;
        packet.x = (int16_t)(SCREEN_WIDTH / 2 + amplitude * sin(lissajous_freq_a * lissajous_time + lissajous_phase));
        packet.y = (int16_t)(SCREEN_HEIGHT / 2 + amplitude * sin(lissajous_freq_b * lissajous_time));
        if (points_queue != NULL) {
            xQueueSend(points_queue, &packet, 0); // Envoi sans bloquer
        }
    }
}

// =======================================================
// TÂCHE 2 : Listener SPI
// =======================================================
static void spi_listener_task(void *pvParameters) {
    esp_err_t ret;

    // Configuration du bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1, // Non utilisé pour SPI esclave standard
        .quadhd_io_num = -1, // Non utilisé pour SPI esclave standard
        // max_transfer_sz est défini par le pilote SPI ESP-IDF et doit être au moins de 32 octets pour le DMA.
        // La taille est gérée par `trans.length` dans `spi_slave_transmit`.
    };

    // Configuration de l'interface esclave
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = GPIO_CS,
        .flags = 0, // Pas de flags spécifiques pour le moment (ex: LSBFIRST)
        .queue_size = 3, // On peut préparer 3 transactions à l'avance
        .mode = 0, // SPI Mode 0: CPOL=0, CPHA=0
        .post_setup_cb = NULL, // Pas de callback après setup
        .post_trans_cb = post_trans_cb, // Notre ISR pour traiter la transaction terminée
    };

    // Initialisation du pilote SPI esclave
    // Nous utilisons SPI2_HOST et SPI_DMA_CH_AUTO pour laisser le système choisir le canal DMA.
    ret = spi_slave_initialize(SPI_HOST_ID, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

    // Enregistrement du statut d'initialisation
    g_spi_initialized_ok = (ret == ESP_OK);

    if (ret != ESP_OK) {
        DEBUG_ERRORF("Erreur lors de l'initialisation du SPI esclave : %d\n", ret);
        // Dans une application réelle, on pourrait gérer cette erreur de manière plus robuste.
        // Pour ce projet, on marque simplement que l'initialisation a échoué.
        // La tâche continuera, mais g_spi_initialized_ok sera false.
    }

    // Boucle de "transaction perpétuelle"
    for (;;) {
        // Préparer la transaction. On dit au pilote qu'on attend un paquet de la taille de PlotterPacket.
        spi_slave_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length = sizeof(PlotterPacket) * 8; // La longueur est en BITS
        trans.rx_buffer = spi_rx_buffer;
        trans.user = NULL; // Pas de données utilisateur spécifiques pour cette transaction

        // "Armer" le récepteur. Cette fonction bloque jusqu'à ce que le maître initie une transaction.
        // Elle utilise le canal DMA configuré.
        ret = spi_slave_transmit(SPI_HOST_ID, &trans, portMAX_DELAY); // Attendre indéfiniment une transaction

        if (ret != ESP_OK) {
            // Si spi_slave_transmit échoue, c'est une erreur système sérieuse.
            // On pourrait vouloir signaler cela plus gravement, mais pour l'instant, on enregistre seulement.
            DEBUG_ERRORF("Erreur lors de la transmission SPI : %d\n", ret);
            // Il pourrait être nécessaire de réinitialiser le périphérique SPI dans un cas réel.
        }
        // Si ret == ESP_OK, la transaction est terminée et l'ISR `post_trans_cb` a déjà été appelée.
        // La boucle recommence pour armer la transaction suivante.
    }
}


// --- IMPLÉMENTATION DE L'INTERFACE PUBLIQUE ---
void start_data_source_tasks(TaskHandle_t* outLissajousHandle, TaskHandle_t* outSpiHandle) {
    // Création de la tâche Lissajous sur le coeur 0
    xTaskCreatePinnedToCore(
        lissajous_task, "Lissajous", 4096, NULL, 2, outLissajousHandle, 0);

    // Création de la tâche SPI Listener sur le coeur 0
    xTaskCreatePinnedToCore(
        spi_listener_task, "SPI Listener", 4096, NULL, 2, outSpiHandle, 0);

    // Par défaut, le mode SPI est actif au démarrage.
    // On suspend la tâche Lissajous.
    if (*outLissajousHandle != NULL) {
        vTaskSuspend(*outLissajousHandle);
    }
}