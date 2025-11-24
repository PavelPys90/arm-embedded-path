/*
 * ============================================================================
 * 1. FILE HEADER
 * ============================================================================
 * @file    : main.c
 * @author  : Dein Name (z.B. Pavel)
 * @version : 1.0
 * @date    : 12.11.2025
 * @brief   : Kurze Beschreibung, was dieses Projekt tut.
 * (z.B. "Projekt 7: Non-Blocking State Machine für LEDs")
 * ============================================================================
 */


/*
 * ============================================================================
 * 2. INCLUDES
 * ============================================================================
 * Hier kommt alles rein, was der Compiler "kennen" muss.
 * Immer zuerst der "Chip-Header", dann die C-Standard-Header.
 */
#include "stm32f103x6.h" // Dein wichtigster Header! (Kennt GPIOC, RCC, etc.)
#include <stdint.h>      // Für uint32_t, uint8_t, etc.
// #include <string.h>   // (Falls du später mal C-Strings brauchst)
// #include "meine_led_api.h" // (Später, wenn du Code in andere .h-Dateien auslagerst)


/*
 * ============================================================================
 * 3. DEFINES & MAKROS (Konstanten)
 * ============================================================================
 * (Alles, was einen Namen statt einer "magischen Zahl" haben soll)
 */
#define LED_BLINK_DELAY_MS   300
#define BUTTON_PIN           GPIO_IDR_IDR0 // (Unser Taster an Pin 0)


/*
 * ============================================================================
 * 4. TYPE DEFINITIONS (Enums, Structs)
 * ============================================================================
 * (Deine eigenen, globalen Datentypen)
 */
typedef enum {
    MODE_CHRISTMAS,
    MODE_ALARM
} App_Mode_t;


/*
 * ============================================================================
 * 5. GLOBALE VARIABLEN
 * ============================================================================
 * (Versuche, so WENIG wie möglich globale Variablen zu nutzen!)
 */

// 5.1. "Externe" Variablen (Definiert in einer ANDEREN .c Datei)
// -----------------------------------------------------------------
// Wir sagen dem Compiler: "g_msTicks existiert, vertrau mir.
// Der Linker findet das später in stm32f1xx_it.c"
extern volatile uint32_t g_msTicks;


// 5.2. "Private" globale Variablen (Nur in DIESER main.c sichtbar)
// -----------------------------------------------------------------
// 'static' macht die Variable "privat" für diese Datei.
// Das ist der "saubere" Weg für globale Variablen.
static App_Mode_t g_current_mode = MODE_CHRISTMAS;
static uint8_t g_last_button_state = 1; // 1 = losgelassen


/*
 * ============================================================================
 * 6. FUNKTIONS-PROTOTYPEN
 * ============================================================================
 * (Das "Inhaltsverzeichnis" für den Compiler)
 *
 * Warum? Der C-Compiler liest von oben nach unten. Wenn main() die Funktion
 * GPIO_Init() aufruft, der Compiler sie aber noch nicht "gesehen" hat,
 * wirft er eine Warnung ("implicit declaration").
 *
 * Hier listen wir alle Funktionen auf, die in dieser Datei existieren.
 */
void GPIO_Init_All(void);
void Button_Check_For_Mode_Change(void);
void LED_Modus_Christmas_Update(void);
void LED_Modus_Alarm_Update(void);


/*
 * ============================================================================
 * 7. MAIN-FUNKTION (Der "Chef" / Eintrittspunkt)
 * ============================================================================
 */
int main(void) {

    /* --- 1. System-Initialisierung --- */
    // (Takt, SysTick, etc. Alles, was die CPU braucht)
    // (SystemInit() wurde schon von der startup_...s aufgerufen)
    SysTick_Config(SystemCoreClock / 1000); // 1ms Tick

    /* --- 2. Peripherie-Initialisierung --- */
    // (Alle "Arbeiter" (Hardware) aufwecken und konfigurieren)
    GPIO_Init_All();

    /* --- 3. Lokale Variablen --- */
    // (Keine in diesem Fall)

    /* --- 4. Die Endlos-Schleife (Der "Scheduler") --- */
    // Das Herz deines Programms. Diese Schleife MUSS schnell sein.
    // Sie darf NIE blockieren.
    while (1)
    {
        // (Delegiere die Arbeit an die "Arbeiter"-Funktionen)

        // Aufgabe 1: Prüfe auf User-Input
        Button_Check_For_Mode_Change();

        // Aufgabe 2: Führe den aktuellen Modus aus
        switch (g_current_mode) {
            case MODE_CHRISTMAS:
                LED_Modus_Christmas_Update();
                break;
            case MODE_ALARM:
                LED_Modus_Alarm_Update();
                break;
        }
    }
} // main() endet hier nie


/*
 * ============================================================================
 * 8. FUNKTIONS-IMPLEMENTIERUNGEN (Die "Arbeiter")
 * ============================================================================
 * (Hier passiert die echte Arbeit. Die Reihenfolge ist egal,
 * da wir oben ja die Prototypen haben.)
 */

/**
 * @brief Initialisiert alle GPIOs für das Projekt.
 * @note  (Doxygen-Stil Kommentare sind super praktisch)
 */
void GPIO_Init_All(void) {
    // ... (Dein RCC- und GPIO-Code für LEDs und Button)
}

/**
 * @brief Prüft den Taster (non-blocking) und schaltet den Modus um.
 */
void Button_Check_For_Mode_Change(void) {
    // ... (Dein "Edge-Detector"-Code mit g_last_button_state)
}

/**
 * @brief State Machine für den Christmas-Modus.
 */
void LED_Modus_Christmas_Update(void) {
    // ... (Dein non-blocking 'static' Code für Christmas)
}

/**
 * @brief State Machine für den Alarm-Modus.
 */
void LED_Modus_Alarm_Update(void) {
    // ... (Dein non-blocking 'static' Code für Alarm)
}

/* --- Ende der Datei --- */
