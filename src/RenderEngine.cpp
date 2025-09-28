/*
 * SPDX-FileCopyrightText: 2023 Adafruit Industries
 * SPDX-License-Identifier: MIT
 *
 * Fichier: RenderEngine.cpp
 * Rôle: Implémentation du moteur de rendu et de la gestion des contrôles.
 * NOTE: Version modifiée pour remplacer le feedback LED par un "Serial BIP".
 */

#include "RenderEngine.h"

// --- VARIABLES PRIVÉES AU MODULE (STATIC) ---

// La définition manuelle de LED_BUILTIN est supprimée car la carte n'a pas de LED standard.
// #define LED_BUILTIN 13

// Matériel
static Arduino_XCA9554SWSPI *expander;
static Arduino_ESP32RGBPanel *rgbpanel;
static Arduino_RGB_Display *gfx;

// Structures de données du moteur
struct RowBounds { int16_t start_x; int16_t width; };
static uint8_t *life_buffer;
static uint16_t render_palette[256];
static RowBounds circle_bounds[720];

// Constantes
static const int SCREEN_WIDTH = 720, SCREEN_HEIGHT = 720;
static const long TOTAL_PIXELS = SCREEN_WIDTH * SCREEN_HEIGHT;
static const float ZOOM_MIN = 0.1f, ZOOM_MAX = 1.2f;
static const long REMANENCE_MS_MIN = 1, REMANENCE_MS_MAX = 5000;

// Variables de contrôle partagées
volatile float g_zoom_factor = 0.8;
volatile float g_remanence_duration_ms = 500;
volatile int g_trace_decay = 4;
volatile bool g_zoom_changed = false;
volatile bool g_remanence_changed = true;

// Lien vers la variable définie dans DataSource.cpp
volatile float g_lissajous_speed;
// Lien vers la variable pour le heartbeat SPI
extern volatile unsigned long g_last_spi_packet_time;

// Variable partagée globale (définie ici)
QueueHandle_t points_queue;

// Variables pour le smoothing des entrées ADC et le suivi des changements
static float last_smoothed_zoom = 0.8; // Déclaration statique pour control_task
static int last_zoom_adc = 0;
static float smoothed_zoom_adc = 0;
static int last_decay_adc = 0;
static float smoothed_decay_adc = 0;

// Flag pour savoir si le SPI a été initialisé correctement (vérité externe)
extern bool g_spi_initialized_ok;


// --- FONCTIONS HELPER PRIVÉES ---

static void precompute_phosphor_palette() {
    const int R_MAX = 31; const int G_MAX = 63;
    for (int life = 0; life < 256; life++) {
        float ratio = life / 255.0f;
        float green_ratio = ratio * ratio;
        uint8_t g = (uint8_t)(G_MAX * green_ratio);
        float red_ratio = sqrt(ratio);
        uint8_t r = (uint8_t)(R_MAX * red_ratio);
        render_palette[life] = (r << 11) | (g << 5);
    }
    render_palette[0] = BLACK;
}

static void precompute_circle_mask() {
    int radius = SCREEN_WIDTH / 2;
    int radius_sq = radius * radius;
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        int dy = y - radius;
        int dx_sq = radius_sq - dy * dy;
        if (dx_sq < 0) {
            circle_bounds[y] = {0, 0};
        } else {
            int dx = sqrt(dx_sq);
            circle_bounds[y].start_x = radius - dx;
            circle_bounds[y].width = 2 * dx;
        }
    }
}

static float catmullRom(float t, float p0, float p1, float p2, float p3) {
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t * t + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t * t * t);
}

static void draw_catmull_rom_segment(PlotterPacket p0, PlotterPacket p1, PlotterPacket p2, PlotterPacket p3) {
    int steps = (int)sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)) / 2;
    steps = max(1, steps);
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float sx = catmullRom(t, p0.x, p1.x, p2.x, p3.x);
        float sy = catmullRom(t, p0.y, p1.y, p2.y, p3.y);
        long screen_index = (long)sy * SCREEN_WIDTH + (long)sx;
        if (screen_index >= 0 && screen_index < TOTAL_PIXELS) {
            life_buffer[screen_index] = 255;
            gfx->drawPixel(sx, sy, render_palette[255]);
        }
    }
}


// --- TÂCHES FREE_RTOS ---

struct ControlTaskParams { TaskHandle_t lissajous_handle; TaskHandle_t spi_handle; };

static void control_task(void *pvParameters) {
    ControlTaskParams* params = (ControlTaskParams*)pvParameters;
    TaskHandle_t lissajousHandle = params->lissajous_handle;
    TaskHandle_t spiHandle = params->spi_handle;

    enum DataSourceMode { MODE_LISSAJOUS, MODE_SPI } current_mode = MODE_LISSAJOUS;

    // Variables pour les timers des messages série
    unsigned long last_bip_time = 0;
    unsigned long last_spi_ok_time = 0;
    const unsigned long BIP_INTERVAL = 2000; // 2 secondes
    const unsigned long SPI_OK_INTERVAL = 2000; // 2 secondes

    // Lecture initiale pour le smoothing
    last_zoom_adc = analogRead(A0);
    smoothed_zoom_adc = last_zoom_adc;
    last_decay_adc = analogRead(A1);
    smoothed_decay_adc = last_decay_adc;

    const int ADC_DEADBAND = 12;
    const float SMOOTH_FACTOR = 0.05;

    // Utilisation de la variable globale g_spi_initialized_ok déclarée dans DataSource.cpp
    // extern bool g_spi_initialized_ok; // Déjà déclaré dans DataSource.h et inclus

    for (;;) {
        unsigned long current_millis = millis();

        // --- GESTION DU BOUTON UP POUR CHANGER DE MODE ---
        if (!expander->digitalRead(PCA_BUTTON_UP)) {
            if (current_mode == MODE_LISSAJOUS) {
                vTaskSuspend(lissajousHandle);
                // Vérifier si la tâche SPI a été initialisée avant de la reprendre
                if (g_spi_initialized_ok) {
                    vTaskResume(spiHandle);
                    current_mode = MODE_SPI;
                    Serial.println("Mode: SPI Actif");
                } else {
                    Serial.println("Mode: SPI non initialisé. Impossible de passer en mode SPI.");
                    // On reste en mode Lissajous ou on réessaie plus tard
                }
            } else { // current_mode == MODE_SPI
                vTaskSuspend(spiHandle);
                vTaskResume(lissajousHandle);
                current_mode = MODE_LISSAJOUS;
                Serial.println("Mode: Lissajous Actif");
            }
            delay(250); // Debounce delay
        }

        // --- GESTION DES MESSAGES PÉRIODIQUES ---
        // Ces messages ne s'affichent que si le mode SPI est actif.
        if (current_mode == MODE_SPI) {
            // BIP: Indique la réception continue de paquets SPI
            // On vérifie si l'activité SPI est récente (dans les 2 dernières secondes).
            if (current_millis - g_last_spi_packet_time < BIP_INTERVAL) {
                if (current_millis - last_bip_time >= BIP_INTERVAL) {
                    Serial.println("[BIP] Reception SPI active.");
                    last_bip_time = current_millis;
                }
            }

            // SPI_OK: Indique que le bus SPI est initialisé et opérationnel
            if (g_spi_initialized_ok) { // Vérifie si l'initialisation a réussi
                if (current_millis - last_spi_ok_time >= SPI_OK_INTERVAL) {
                    Serial.println("[SPI_OK] Bus SPI initialisé et opérationnel.");
                    last_spi_ok_time = current_millis;
                }
            }
        }

        // --- LECTURE DES POTENTIOMÈTRES ---
        int raw_zoom_adc = analogRead(A0);
        // Mise à jour du smoothing pour le zoom
        if (abs(raw_zoom_adc - last_zoom_adc) > ADC_DEADBAND) {
            smoothed_zoom_adc = SMOOTH_FACTOR * raw_zoom_adc + (1 - SMOOTH_FACTOR) * smoothed_zoom_adc;
            last_zoom_adc = raw_zoom_adc;
        }
        int raw_decay_adc = analogRead(A1);
        // Mise à jour du smoothing pour la rémanence
        if (abs(raw_decay_adc - last_decay_adc) > ADC_DEADBAND) {
            smoothed_decay_adc = SMOOTH_FACTOR * raw_decay_adc + (1 - SMOOTH_FACTOR) * smoothed_decay_adc;
            last_decay_adc = raw_decay_adc;
            g_remanence_changed = true;
        }

        // Mise à jour des facteurs de contrôle
        g_zoom_factor = map(smoothed_zoom_adc, 0, 4095, (long)(ZOOM_MIN * 100), (long)(ZOOM_MAX * 100)) / 100.0f;
        g_remanence_duration_ms = map(smoothed_decay_adc, 0, 4095, REMANENCE_MS_MIN, REMANENCE_MS_MAX);

        // Détection de changement pour le zoom (utilisé pour réinitialiser le rendu si nécessaire)
        if (abs(g_zoom_factor - last_smoothed_zoom) > 0.01) {
            g_zoom_changed = true;
            last_smoothed_zoom = g_zoom_factor; // Mise à jour de la dernière valeur affichée/utilisée
        }

        // --- LECTURE DES COMMANDES SÉRIE ---
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            if (cmd == 'p') g_lissajous_speed *= 1.1; // Augmente la vitesse du simulateur Lissajous
            if (cmd == 'm') g_lissajous_speed /= 1.1; // Diminue la vitesse du simulateur Lissajous
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Petite pause pour ne pas saturer le CPU
    }
}

static void render_task(void *pvParameters) {
    uint16_t *framebuffer = (uint16_t *)gfx->getFramebuffer();
    PlotterPacket waypoints[4];
    int waypoint_count = 0;
    unsigned long frameCount = 0;
    unsigned long lastFpsTime = 0; // Déclarée localement dans la fonction
    float last_displayed_fps = 60.0;
    int fade_slice_index = 0;
    const int NUM_FADE_SLICES = 4;

    for (;;) {
        unsigned long current_millis = millis(); // Déclarée localement dans la boucle

        if (g_zoom_changed) {
            gfx->fillScreen(BLACK);
            memset(life_buffer, 0, TOTAL_PIXELS);
            waypoint_count = 0; // Réinitialiser le compteur de points pour le nouveau tracé
            g_zoom_changed = false;
        }

        if (g_remanence_changed) {
            // Recalcul du facteur de déclin basé sur la durée de rémanence et le FPS actuel
            float target_fps = (last_displayed_fps > 0) ? last_displayed_fps : 60.0;
            // Assure que g_trace_decay est au minimum 1 pour éviter une division par zéro ou un déclin trop lent
            g_trace_decay = max(1, (int)(255.0f / ((g_remanence_duration_ms / 1000.0f) * target_fps)));
            g_remanence_changed = false;
        }

        // Boucle de fondu phosphore par tranche entrelacée
        for (int y = fade_slice_index; y < SCREEN_HEIGHT; y += NUM_FADE_SLICES) {
            // Appliquer le masque circulaire en ne parcourant que les pixels valides
            for (int i = 0; i < circle_bounds[y].width; i++) {
                long current_index = (y * SCREEN_WIDTH) + circle_bounds[y].start_x + i;
                // Vérifier si le pixel a encore de la "vie"
                if (life_buffer[current_index] > 0) {
                    // Décrémenter la vie du pixel (appliquer le déclin)
                    life_buffer[current_index] = (life_buffer[current_index] > g_trace_decay) ? life_buffer[current_index] - g_trace_decay : 0;
                    // Mettre à jour le pixel dans le framebuffer graphique avec la couleur correspondante
                    framebuffer[current_index] = render_palette[life_buffer[current_index]];
                }
            }
        }
        // Passer à la tranche suivante pour le prochain rendu d'image
        fade_slice_index = (fade_slice_index + 1) % NUM_FADE_SLICES;

        // Traitement des nouveaux waypoints reçus via la queue
        PlotterPacket p_raw;
        while (xQueueReceive(points_queue, &p_raw, 0) == pdTRUE) {
            PlotterPacket p_zoomed;
            // Appliquer le facteur de zoom
            p_zoomed.x = (int16_t)(SCREEN_WIDTH / 2 + (p_raw.x - SCREEN_WIDTH / 2) * g_zoom_factor);
            p_zoomed.y = (int16_t)(SCREEN_HEIGHT / 2 + (p_raw.y - SCREEN_HEIGHT / 2) * g_zoom_factor);

            // Mettre à jour le buffer de waypoints pour le calcul de la courbe spline
            waypoints[0] = waypoints[1]; waypoints[1] = waypoints[2];
            waypoints[2] = waypoints[3]; waypoints[3] = p_zoomed;

            if (waypoint_count < 4) waypoint_count++; // Incrémenter seulement si on n'a pas encore 4 points

            // Dessiner le segment de courbe spline si nous avons assez de points
            if (waypoint_count == 4) {
                draw_catmull_rom_segment(waypoints[0], waypoints[1], waypoints[2], waypoints[3]);
            }
        }

        // Calcul et affichage des FPS toutes les 500ms
        frameCount++;
        if (current_millis - lastFpsTime >= 500) {
            last_displayed_fps = frameCount * (1000.0f / (current_millis - lastFpsTime));
            Serial.printf("FPS: %.0f\n", last_displayed_fps);
            lastFpsTime = current_millis;
            frameCount = 0;
        }
    }
}


// --- IMPLÉMENTATION DE L'INTERFACE PUBLIQUE ---

void start_render_and_control_tasks(TaskHandle_t lissajousHandle, TaskHandle_t spiHandle) {
    static ControlTaskParams params;
    params.lissajous_handle = lissajousHandle;
    params.spi_handle = spiHandle;

    xTaskCreatePinnedToCore(control_task, "Controls", 4096, &params, 0, NULL, 0);
    xTaskCreatePinnedToCore(render_task, "Render", 16384, NULL, 1, NULL, 1);
}

void setup_renderer_and_controls() {
    expander = new Arduino_XCA9554SWSPI(PCA_TFT_RESET, PCA_TFT_CS, PCA_TFT_SCK, PCA_TFT_MOSI, &Wire, 0x3F);
    rgbpanel = new Arduino_ESP32RGBPanel(
        TFT_DE, TFT_VSYNC, TFT_HSYNC, TFT_PCLK,
        TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_R5,
        TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
        TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_B5,
        1, 46, 2, 44, 1, 50, 16, 16, 1, 16000000);
    gfx = new Arduino_RGB_Display(
        720, 720, rgbpanel, 0, true,
        expander, GFX_NOT_DEFINED, hd40015c40_init_operations, sizeof(hd40015c40_init_operations));

    if (!gfx->begin()) { while (1); }
    expander->pinMode(PCA_TFT_BACKLIGHT, OUTPUT);
    expander->digitalWrite(PCA_TFT_BACKLIGHT, HIGH);
    expander->pinMode(PCA_BUTTON_UP, INPUT);
    g_lissajous_speed = 0.05f; // Valeur par défaut pour que le mouvement soit visible
    life_buffer = (uint8_t *)ps_malloc(TOTAL_PIXELS * sizeof(uint8_t));
    if (!life_buffer) { while(1); } // Allocation échouée, bloquer le système
    memset(life_buffer, 0, TOTAL_PIXELS); // Initialiser le buffer de vie à zéro

    precompute_phosphor_palette();
    precompute_circle_mask();
    gfx->fillScreen(BLACK); // Effacer l'écran au démarrage

    // Création de la file d'attente pour les points à afficher
    points_queue = xQueueCreate(1024, sizeof(PlotterPacket));
    if (points_queue == NULL) { while(1); } // Création de la queue échouée, bloquer le système
}