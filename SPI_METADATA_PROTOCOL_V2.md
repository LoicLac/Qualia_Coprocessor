# Protocole SPI v2 - Metadata Display (Engine & Sonifier)

**Date:** 14 octobre 2025  
**Version:** 2.0  
**Auteur:** Architecture DaisyEquation  
**Destinataire:** Développeur du coprocesseur graphique (Raspberry Pi/ESP32)

---

## 🎯 Objectif

Le Daisy Seed envoie maintenant **2 types de paquets SPI** :
1. **Paquets de coordonnées XY** (envoi continu à 1 kHz)
2. **Paquets de metadata** (envoi uniquement lors des changements d'engine ou de sonifier)

Cela permet au coprocesseur graphique d'afficher le nom de l'équation active et du sonifieur actif.

---

## 📦 Structure des Paquets

### Type 1: Paquet de Coordonnées (4 bytes)

```c
struct __attribute__((packed)) PlotterPacket {
    uint16_t x;  // Coordonnée X écran (0-719 pour affichage 720x720)
    uint16_t y;  // Coordonnée Y écran (0-719 pour affichage 720x720)
};
```

**Fréquence:** ~1000 Hz (1 ms entre chaque envoi)  
**Identification:** Le premier byte n'est **jamais** `0xFF`

### Type 2: Paquet de Metadata (4 bytes)

```c
struct __attribute__((packed)) MetadataPacket {
    uint8_t packet_type;     // TOUJOURS 0xFF (identifiant du type metadata)
    uint8_t engine_index;    // Index de l'engine actif (0-6)
    uint8_t sonifier_index;  // Index du sonifier actif (0-5)
    uint8_t reserved;        // Réservé pour extension future
};
```

**Fréquence:** Uniquement lors d'un changement (sporadique)  
**Identification:** Le premier byte vaut **exactement** `0xFF`

---

## 🔍 Logique de Réception

### Pseudo-code de Parser

```c
uint8_t spi_buffer[4];

// Recevoir 4 bytes via SPI
spi_receive(spi_buffer, 4);

// Distinction du type de paquet
if (spi_buffer[0] == 0xFF) {
    // === PAQUET METADATA ===
    uint8_t engine_idx = spi_buffer[1];
    uint8_t sonifier_idx = spi_buffer[2];
    
    update_display_labels(engine_idx, sonifier_idx);
    
} else {
    // === PAQUET COORDONNÉES XY ===
    uint16_t x = (uint16_t)(spi_buffer[0] | (spi_buffer[1] << 8));
    uint16_t y = (uint16_t)(spi_buffer[2] | (spi_buffer[3] << 8));
    
    plot_point(x, y);
}
```

### Exemple Arduino/ESP32

```cpp
#include <SPI.h>

// Déclarations
const char* current_engine_name = "Lissajous";
const char* current_sonifier_name = "Stereo XY";

void loop() {
    if (SPI.available() >= 4) {
        uint8_t buffer[4];
        
        // Recevoir 4 bytes
        for (int i = 0; i < 4; i++) {
            buffer[i] = SPI.transfer(0x00);
        }
        
        // Parser le paquet
        if (buffer[0] == 0xFF) {
            // Metadata reçue
            handleMetadata(buffer[1], buffer[2]);
        } else {
            // Coordonnées XY
            uint16_t x = buffer[0] | (buffer[1] << 8);
            uint16_t y = buffer[2] | (buffer[3] << 8);
            drawPixel(x, y);
        }
    }
}

void handleMetadata(uint8_t engine_idx, uint8_t sonifier_idx) {
    // Mettre à jour les labels
    current_engine_name = getEngineName(engine_idx);
    current_sonifier_name = getSonifierName(sonifier_idx);
    
    // Rafraîchir l'affichage (zone de texte, OLED, LCD, etc.)
    updateDisplayLabels();
}
```

---

## 📋 Tables de Mapping

### Engines (7 équations disponibles)

| Index | Nom Technique       | Nom d'Affichage Suggéré  |
|-------|---------------------|---------------------------|
| 0     | LissajousEngine     | Lissajous                 |
| 1     | RosePolarEngine     | Rose Polaire              |
| 2     | SuperellipseEngine  | Superellipse              |
| 3     | TrochoideEngine     | Trochoïde                 |
| 4     | PolygonEngine       | Polygone Régulier         |
| 5     | StarPolygonEngine   | Polygone Étoilé           |
| 6     | HarmonicXYEngine    | Harmonique XY             |

### Sonifiers (6 méthodes de sonification)

| Index | Nom Technique              | Nom d'Affichage Suggéré   |
|-------|----------------------------|---------------------------|
| 0     | StereoXYSonifier           | Stereo XY                 |
| 1     | TangentialSonifier         | Tangentiel                |
| 2     | MonoRadiusSonifier         | Rayon Mono                |
| 3     | CrossDimensionSonifier     | Cross Dimension           |
| 4     | ObliqueProjectionSonifier  | Projection Oblique        |
| 5     | DistanceSonifier           | Distance                  |

### Implémentation C/C++

```c
const char* getEngineName(uint8_t index) {
    static const char* names[] = {
        "Lissajous",
        "Rose Polaire",
        "Superellipse",
        "Trochoide",
        "Polygone Regulier",
        "Polygone Etoile",
        "Harmonique XY"
    };
    return (index < 7) ? names[index] : "Unknown";
}

const char* getSonifierName(uint8_t index) {
    static const char* names[] = {
        "Stereo XY",
        "Tangentiel",
        "Rayon Mono",
        "Cross Dimension",
        "Projection Oblique",
        "Distance"
    };
    return (index < 6) ? names[index] : "Unknown";
}
```

---

## 🎨 Recommandations d'Affichage

### Layout Suggéré

```
┌─────────────────────────────────────────┐
│  Engine: Lissajous                      │
│  Sonifier: Tangentiel                   │
├─────────────────────────────────────────┤
│                                         │
│          [Zone de tracé 720x720]        │
│                                         │
│                                         │
└─────────────────────────────────────────┘
```

### Gestion du Refresh

- **Zone de tracé:** Rafraîchir en continu (recevoir ~1000 points/seconde)
- **Zone de texte:** Mettre à jour **uniquement** à la réception d'un paquet metadata
- **Éviter:** Ne pas effacer le tracé lors d'un changement de label (sauf si intentionnel)

### Options d'Affichage

1. **Affichage persistant:** Texte toujours visible en haut de l'écran
2. **Overlay temporaire:** Afficher pendant 2 secondes lors d'un changement, puis fade out
3. **Indicateur LED:** Clignoter une LED pour signaler un changement
4. **Affichage externe:** OLED séparé ou écran LCD pour les metadata

---

## ⚙️ Configuration SPI (Côté Récepteur)

### Paramètres SPI du Daisy Seed (Émetteur)

```
Clock Speed: 20 MHz
Mode: SPI_MODE0 (CPOL=0, CPHA=0)
Bit Order: MSBFIRST
CS Pin: PIN_SPI_SS (actif LOW)
```

### Configuration Récepteur (Exemple ESP32)

```cpp
#include <SPI.h>

SPIClass* hspi = new SPIClass(HSPI);

void setup() {
    hspi->begin(
        14,  // SCK
        12,  // MISO (connecté au MOSI du Daisy)
        13,  // MOSI (non utilisé en slave)
        15   // CS
    );
    
    hspi->beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    pinMode(15, INPUT);  // CS comme entrée (contrôlé par le Daisy)
}
```

**Note:** Le Daisy est **maître SPI**, le coprocesseur graphique est **esclave**.

---

## 🔧 Debugging et Validation

### Test de Réception

```cpp
void debug_packet(uint8_t* buffer) {
    if (buffer[0] == 0xFF) {
        Serial.printf("METADATA: Engine=%d, Sonifier=%d\n", 
                      buffer[1], buffer[2]);
    } else {
        uint16_t x = buffer[0] | (buffer[1] << 8);
        uint16_t y = buffer[2] | (buffer[3] << 8);
        Serial.printf("PLOT: X=%d, Y=%d\n", x, y);
    }
}
```

### Validation des Indices

```cpp
bool validate_metadata(uint8_t engine_idx, uint8_t sonifier_idx) {
    if (engine_idx >= 7) {
        Serial.println("ERROR: Invalid engine index");
        return false;
    }
    if (sonifier_idx >= 6) {
        Serial.println("ERROR: Invalid sonifier index");
        return false;
    }
    return true;
}
```

---

## 📊 Statistiques de Trafic

| Type de Paquet | Fréquence      | Bandwidth            |
|----------------|----------------|----------------------|
| Coordonnées XY | 1000 Hz        | 32 kbit/s (4 bytes)  |
| Metadata       | Sporadique     | Négligeable          |
| **Total**      | **~1000 Hz**   | **~32 kbit/s**       |

**Note:** Les paquets metadata représentent < 0.1% du trafic SPI total.

---

## ✅ Checklist d'Implémentation

- [ ] Parser les paquets entrants (distinction via `buffer[0] == 0xFF`)
- [ ] Implémenter `getEngineName()` avec les 7 noms
- [ ] Implémenter `getSonifierName()` avec les 6 noms
- [ ] Créer une zone d'affichage pour les labels (texte ou OLED)
- [ ] Tester la réception avec un changement d'engine sur le Daisy
- [ ] Tester la réception avec un changement de sonifier
- [ ] Valider que le tracé XY continue pendant les changements
- [ ] Valider les indices (0-6 pour engines, 0-5 pour sonifiers)

---

## 🆘 Support et Contact

**Fichier de référence (Daisy Seed):**  
`include/hardware/shared_protocol.h` (version 2)

**Questions fréquentes:**

**Q: Les coordonnées peuvent-elles avoir `x=0xFF` ?**  
R: Oui, mais le paquet complet sera `[0xFF, 0x00, y_low, y_high]`. Pour identifier un metadata packet, vérifier `buffer[0] == 0xFF` **ET** `buffer[1] < 7` (engine index valide).

**Q: Faut-il acquitter les paquets ?**  
R: Non, le protocole est unidirectionnel (Daisy → Coprocesseur). Pas d'ACK requis.

**Q: Que faire si je reçois un index invalide ?**  
R: Afficher "Unknown" ou garder le dernier nom valide. Logger l'erreur.

**Q: Les paquets peuvent-ils arriver dans le désordre ?**  
R: Non, SPI est synchrone. L'ordre est garanti.

---

## 📝 Changelog

### Version 2.0 (Octobre 2025)
- Ajout des paquets metadata (engine + sonifier)
- Identification par flag `0xFF` dans le premier byte
- Tables de mapping pour les 7 engines et 6 sonifiers

### Version 1.0 (Initial)
- Paquets XY uniquement (4 bytes)
- Fréquence 1 kHz

---

**Bonne implémentation !** 🚀

